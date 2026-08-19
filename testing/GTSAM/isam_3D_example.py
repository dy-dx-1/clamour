import gtsam as gt
import gtsam.utils.plot as gtsam_plot
import matplotlib.pyplot as plt
from random import gauss

### Noise
ANCHOR_POS_NOISE = gt.noiseModel.Diagonal.Sigmas([0.02, 0.02, 0.02])  # 3D now: x, y, z
# Pose3 tangent space is 6D: [rotation(rx,ry,rz), translation(tx,ty,tz)]
ODOMETRY_NOISE = gt.noiseModel.Diagonal.Sigmas([0.05, 0.05, 0.05, 0.3, 0.3, 0.3])
RANGE_NOISE = gt.noiseModel.Isotropic.Sigma(1, 0.1)  # still a scalar range, unchanged

# motion: (translation (dx,dy,dz), rotation as (roll,pitch,yaw))
# anchors: now 3D positions
motion_and_measurements = {
    1: (None, {'a_ids': [1, 2, 3],
               'a_pos': [(0, -1, 0), (0, 1, 0), (-1, 0, 0)],
               'zs': [1 + gauss(0, 0.1) for _ in range(3)]}),
    2: (((1, 0, 0), (0, 0, 0)), None),
    3: (((1, 1, 0), (0, 0, 0)), None),
    4: (((1, 1, 0), (0, 0, 0)), {'a_ids': [4], 'a_pos': [(3, 3, 0)],
                                   'zs': [1 + gauss(0, 0.1)]}),
    5: (((1, 0, 0), (0, 0, 0)), {'a_ids': [5, 6, 7],
                                   'a_pos': [(4, 3, 0), (5, 2, 0), (4, 1, 0)],
                                   'zs': [1 + gauss(0, 0.1) for _ in range(3)]}),
}

### Starting ISAM
isam = gt.ISAM2()
results = None

for state_id, (motion, measurements) in list(motion_and_measurements.items())[:]:
    graph = gt.NonlinearFactorGraph()
    initial = gt.Values()

    x = gt.symbol('x', state_id)
    if motion:
        x_prev = gt.symbol('x', state_id - 1)
        trans, rot = motion
        rotation = gt.Rot3.Ypr(rot[2], rot[1], rot[0])  # yaw, pitch, roll
        mvt = gt.Pose3(rotation, gt.Point3(trans[0], trans[1], trans[2]))
        graph.add(gt.BetweenFactorPose3(x_prev, x, mvt, ODOMETRY_NOISE))

        previous_pose = results.atPose3(x_prev)
        initial.insert(x, previous_pose.compose(mvt))
    else:
        initial.insert(x, gt.Pose3())  # identity pose = origin, no rotation
        graph.add(gt.PriorFactorPose3(
            x, gt.Pose3(),
            gt.noiseModel.Diagonal.Sigmas([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        ))

    if measurements:
        for a_id, a_pos, z in zip(measurements['a_ids'], measurements['a_pos'], measurements['zs']):
            l = gt.symbol('l', a_id)
            graph.add(gt.PriorFactorPoint3(l, gt.Point3(*a_pos), ANCHOR_POS_NOISE))
            graph.add(gt.RangeFactor3D(x, l, z, RANGE_NOISE))
            initial.insert(l, gt.Point3(*a_pos))

    isam.update(graph, initial)
    results = isam.calculateEstimate()

### plot estimates
fig = plt.figure(0)
ax = fig.add_subplot(projection='3d')

for key in results.keys():
    cov_matrix = isam.marginalCovariance(key)
    print(f"Covariance matrix for state {key}", cov_matrix, sep="\n")
    if gt.Symbol(key).chr() == ord('x'):
        gtsam_plot.plot_pose3(0, results.atPose3(key), 0.5, cov_matrix)
    else:
        gtsam_plot.plot_point3(0, results.atPoint3(key), cov_matrix)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.savefig('isam_trilat_3d.png')
print("RESULTS:", results, sep="\n")