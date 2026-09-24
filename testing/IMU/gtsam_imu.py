import numpy as np
import gtsam 
from gtsam.symbol_shorthand import B, V, X
from matplotlib import pyplot as plt

GRAVITY_METERS_PER_SECOND_SQUARED = 9.81
TIMESTAMP_TICK_SECONDS = 21.7e-6
MG_TO_METERS_PER_SECOND_SQUARED = GRAVITY_METERS_PER_SECOND_SQUARED * 1e-3
MDPS_TO_RADIANS_PER_SECOND = np.pi / (180.0 * 1000.0)

### Setting up preintegration -----------------
# pim: preintegrated measurements 
# define Z axis pointing up. factor will handle gravity internally 
pim_params = gtsam.PreintegrationCombinedParams.MakeSharedU(GRAVITY_METERS_PER_SECOND_SQUARED)
# Some arbitrary noise sigmas for accel, gyro and numerical integration
pim_params.setGyroscopeCovariance((3.8 * MDPS_TO_RADIANS_PER_SECOND)**2 * np.eye(3))
pim_params.setAccelerometerCovariance((0.060 * MG_TO_METERS_PER_SECOND_SQUARED)**2 * np.eye(3))
pim_params.setIntegrationCovariance((1e-7)**2 * np.eye(3))
pim_params.setBiasAccCovariance((0.032*MG_TO_METERS_PER_SECOND_SQUARED)**2 * np.eye(3)) 
pim_params.setBiasOmegaCovariance((5.73*MDPS_TO_RADIANS_PER_SECOND)**2 * np.eye(3))  
# Defining IMU biaises 
accBias = np.array([-8.30158247e+00, -2.40556397e-01, -2.6589320093328133])*MG_TO_METERS_PER_SECOND_SQUARED # NOTE biases will be SUBSTRACTED from readings (coherent with theory/def of 'bias')
gyroBias = np.array([-358.52207506, -13.35871965, -214.78256071])*MDPS_TO_RADIANS_PER_SECOND
imu_bias = gtsam.imuBias.ConstantBias(accBias, gyroBias)
# Creating preintegration object 
# requires sensor cov, initial biais estimate (drift will be estimated afterwards)
# Using combined version as we'll use CombinedImuFactor later
pim = gtsam.PreintegratedCombinedMeasurements(pim_params, imu_bias)

### Integrating raw IMU data -----------------
import csv
positions = []
position_timestamps = []
raw_accelerations = []
raw_gyroscopes = []
raw_timestamps = []
# IMU_testing.py saves timestamp ticks, mg, and mdps. GTSAM expects seconds, m/s^2, and rad/s respectively.
previous_timestamp = None
prev_state = gtsam.NavState(gtsam.Pose3(), np.zeros(3))
with open("straight_line.csv", newline="", encoding="utf-8") as csv_file:
    reader = csv.reader(csv_file)
    next(reader, None)  # Skip the timestamp, accel, gyro header.
    for row in reader:
        timestamp, accel, gyro = row
        if not timestamp or not accel or not gyro:
            continue  # Preserve the CSV export's incomplete-sample handling.
        timestamp = int(timestamp)
        accel_reading = np.fromstring(accel.strip("[]"), sep=" ")
        gyro_reading = np.fromstring(gyro.strip("[]"), sep=" ")
        if accel_reading.size != 3 or gyro_reading.size != 3:
            raise ValueError(f"Invalid IMU vector in CSV row: {row}")
        raw_accelerations.append(accel_reading)
        raw_gyroscopes.append(gyro_reading)
        raw_timestamps.append(timestamp * TIMESTAMP_TICK_SECONDS)
        if previous_timestamp is not None:
            dt = (timestamp - previous_timestamp) * TIMESTAMP_TICK_SECONDS
            if dt <= 0:
                raise ValueError(f"Non-increasing IMU timestamp in CSV row: {row}")
            pim.integrateMeasurement(
                accel_reading * MG_TO_METERS_PER_SECOND_SQUARED,
                gyro_reading * MDPS_TO_RADIANS_PER_SECOND,
                dt,
            )
            positions.append(pim.predict(prev_state, imu_bias).pose().translation())
            position_timestamps.append(timestamp * TIMESTAMP_TICK_SECONDS)
        previous_timestamp = timestamp

if positions:
    position_values = np.asarray(positions)
    raw_acceleration_values = np.asarray(raw_accelerations)
    raw_gyroscope_values = np.asarray(raw_gyroscopes)
    figure, axes = plt.subplots(1, 3, figsize=(18, 5), sharex=True)

    axes[0].plot(position_timestamps, position_values[:, 0], label="X")
    axes[0].plot(position_timestamps, position_values[:, 1], label="Y")
    axes[0].plot(position_timestamps, position_values[:, 2], label="Z")
    axes[0].set_xlabel("Timestamp (s)")
    axes[0].set_ylabel("Position (m)")
    axes[0].set_title("Estimated Position")
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(raw_timestamps, raw_acceleration_values[:, 0], label="X")
    axes[1].plot(raw_timestamps, raw_acceleration_values[:, 1], label="Y")
    axes[1].plot(raw_timestamps, raw_acceleration_values[:, 2]-1000+2.6589320093328133, label="Z")
    axes[1].set_xlabel("Timestamp (s)")
    axes[1].set_ylabel("Acceleration (mg)")
    axes[1].set_title("Raw Accelerations")
    axes[1].legend()
    axes[1].grid(True)

    axes[2].plot(raw_timestamps, raw_gyroscope_values[:, 0], label="X")
    axes[2].plot(raw_timestamps, raw_gyroscope_values[:, 1], label="Y")
    axes[2].plot(raw_timestamps, raw_gyroscope_values[:, 2], label="Z")
    axes[2].set_xlabel("Timestamp (s)")
    axes[2].set_ylabel("Angular velocity (mdps)")
    axes[2].set_title("Raw Gyroscope")
    axes[2].legend()
    axes[2].grid(True)

    figure.tight_layout()
    figure.savefig("positions.png")

### Building graph -----------------
graph = gtsam.NonlinearFactorGraph() 
initial = gtsam.Values() 

pose_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.01]*3 + [0.01]*3))
vel_noise = gtsam.noiseModel.Isotropic.Sigma(3, 0.01)
bias_noise = gtsam.noiseModel.Isotropic.Sigma(6, 0.1)

# Priors state 0 
graph.add(gtsam.PriorFactorPose3(X(0), gtsam.Pose3(), pose_noise))
graph.add(gtsam.PriorFactorVector(V(0), np.zeros(3), vel_noise)) # IMPORTANT NOTE, adding imu means we have to explicitly track velocity and biais estimates
graph.add(gtsam.PriorFactorConstantBias(B(0), imu_bias, bias_noise))
initial.insert(X(0), gtsam.Pose3())
initial.insert(V(0), np.zeros(3))
initial.insert(B(0), imu_bias)

# Using CombinedImuFactor, which removes the need of an independent BetweenFactor to track the bias 
# while keeping taking into account correlations between bias drift and IMU predictions 
imu_factor = gtsam.CombinedImuFactor(X(0), V(0), X(1), V(1), B(0), B(1), pim)
graph.add(imu_factor)

# Initial guess for state 1 (using IMU's own prediction as starting point) 
# using NavState as it encodes pose + velocity, which we need to track with the imu 
prev_state = gtsam.NavState(gtsam.Pose3(), np.zeros(3))
predicted_state = pim.predict(prev_state, imu_bias)
initial.insert(X(1), predicted_state.pose())
initial.insert(V(1), predicted_state.velocity())
initial.insert(B(1), imu_bias)
# Adding zero velocity prior at the end to check if closes better in straight line test (23sept) 
#graph.add(gtsam.PriorFactorVector(V(1), np.zeros(3), vel_noise))
# NOTE after creating IMU factor, clear out preintegration values 
pim.resetIntegration()

# Solving -----------------
optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial)
result = optimizer.optimize()
print("Estimated pose at state 1:", result.atPose3(X(1)))
print("Estimated velocity at state 1:", result.atVector(V(1)))
print("Estimated bias at state 1:", result.atConstantBias(B(1)))