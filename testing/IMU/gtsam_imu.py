import numpy as np
import gtsam 
from gtsam.symbol_shorthand import B, V, X
from matplotlib import pyplot as plt

### Setting up preintegration -----------------
# pim: preintegrated measurements 
# define Z axis pointing up. factor will handle gravity internally 
pim_params = gtsam.PreintegrationParams.MakeSharedU(9.81)
# Some arbitrary noise sigmas for accel, gyro and numerical integration
pim_params.setGyroscopeCovariance((1e-3)**2 * np.eye(3))
pim_params.setAccelerometerCovariance((1e-3)**2 * np.eye(3))
pim_params.setIntegrationCovariance((1e-7)**2 * np.eye(3))
# Defining IMU biaises 
accBias = np.array( [0.0, 0.0, 0.0]) # NOTE biases will be SUBSTRACTED from readings (coherent with theory/def of 'bias')
gyroBias = np.array([0.0, 0.0, 0.0])
imu_bias = gtsam.imuBias.ConstantBias(accBias, gyroBias)
# Creating preintegration object 
# requires sensor cov, initial biais estimate (will be estimated afterwards), optinal transform bodyPsensor for IMU-body frame if needed
pim = gtsam.PreintegratedImuMeasurements(pim_params, imu_bias)

### Integrating raw IMU data -----------------
accel_reading = np.array([1.0, 0.0, 9.81]) # accelerating 1m/s^2 forward, z up 
gyro_reading  = np.array([0.0, 0.0, 0.0]) # NOTE CHECK IF FORCE FLOAT 
dt = 0.01 # 100Hz 
for _ in range(100): # for 1s, at 100Hz, moving fwd at 1m/s^2 
    pim.integrateMeasurement(accel_reading, gyro_reading, dt)

### Building graph -----------------
graph = gtsam.NonlinearFactorGraph() 
initial = gtsam.Values() 

pose_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.01]*3 + [0.01]*3))
vel_noise = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)
bias_noise = gtsam.noiseModel.Isotropic.Sigma(6, 0.001)

# Priors state 0 
graph.add(gtsam.PriorFactorPose3(X(0), gtsam.Pose3(), pose_noise))
graph.add(gtsam.PriorFactorVector(V(0), np.zeros(3), vel_noise)) # IMPORTANT NOTE, adding imu means we have to explicitly track velocity and biais estimates
graph.add(gtsam.PriorFactorConstantBias(B(0), imu_bias, bias_noise))
initial.insert(X(0), gtsam.Pose3())
initial.insert(V(0), np.zeros(3))
initial.insert(B(0), imu_bias)

# IMU Factor between state 0 and 1 
# NOTE THIS IS WHAT WE SHOULD COMPARE AFTERWARDS! DIFFERENT TYPES 
imu_factor = gtsam.ImuFactor(X(0), V(0), X(1), V(1), B(0), pim)
graph.add(imu_factor)
# Bias random walk factor: we set it so bias doesn't change much between states (we expect it to change very slowly)
graph.add(gtsam.BetweenFactorConstantBias(B(0), B(1), gtsam.imuBias.ConstantBias(), bias_noise))

# Initial guess for state 1 (using IMU's own prediction as starting point) 
# using NavState as it encodes pose + velocity, which we need to track with the imu 
prev_state = gtsam.NavState(gtsam.Pose3(), np.zeros(3))
predicted_state = pim.predict(prev_state, imu_bias)
initial.insert(X(1), predicted_state.pose())
initial.insert(V(1), predicted_state.velocity())
initial.insert(B(1), imu_bias)
# NOTE after creating IMU factor, clear out preintegration values 
pim.resetIntegration()

# Solving -----------------
optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial)
result = optimizer.optimize()
print("Estimated pose at state 1:", result.atPose3(X(1)))
print("Estimated velocity at state 1:", result.atVector(V(1)))
print("Estimated bias at state 1:", result.atConstantBias(B(1)))