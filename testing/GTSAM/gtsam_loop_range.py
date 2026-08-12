from math import pi
import gtsam as gt 
import gtsam.utils.plot as gtsam_plot 
import matplotlib.pyplot as plt 

# 2D + rotation problem with odometry in between poses, 1 range measurement and a loop closure 
# Each displacement covers ~1m 

PRIOR_MEAN = gt.Pose2(0, 0, 0) 
PRIOR_NOISE = gt.noiseModel.Diagonal.Sigmas([0.05, 0.05, 0.3]) 
ODOMETRY_NOISE = gt.noiseModel.Diagonal.Sigmas([0.3, 0.3, 0.15]) 

anchor_key = gt.symbol('l', 1) 
RANGE_NOISE = gt.noiseModel.Isotropic.Sigma(1, 0.1) # precise measurement 

graph = gt.NonlinearFactorGraph() 
# Initial factor
graph.add(gt.PriorFactorPose2(1, PRIOR_MEAN, PRIOR_NOISE)) 
# Odometry movements
o12 = gt.Pose2(1, 0, 0) 
o23 = gt.Pose2( 1, 1, 0) 
o34 = gt.Pose2(-1, 1, 0) 
o45 = gt.Pose2(-1, 0, 0)
o51 = gt.Pose2(0, -1, 0) # loop closure 

for key, o_mean in enumerate([o12, o23, o34, o45]): 
    graph.add(gt.BetweenFactorPose2(key+1, key+2, o_mean, ODOMETRY_NOISE))
graph.add(gt.BetweenFactorPose2(5, 1, o51, ODOMETRY_NOISE)) # loop closure 

## Adding range measurement at x3 
measurement = 1 # 1m away
####### important NOTE! anchors need to be added as Point2 factors, means and (later) values! 
graph.add(gt.PriorFactorPoint2(anchor_key, gt.Point2(3, 1), gt.noiseModel.Diagonal.Sigmas([0.05, 0.05])))
graph.add(gt.RangeFactor2D(3, anchor_key, measurement, RANGE_NOISE))

print("GRAPH:", graph, sep="\n")

### Solving 
initial = gt.Values() 
initial.insert(1, gt.Pose2(-0.5, 0.1, 0.2)) # deliberate bad values 
initial.insert(2, gt.Pose2(1.1, 0.0, 0.1))
initial.insert(3, gt.Pose2(2.2, 0.7, 0.0))
initial.insert(4, gt.Pose2(0.7, 1.3, -0.1))
initial.insert(5, gt.Pose2(0.3, 0.7, 0.2))
initial.insert(anchor_key, gt.Point2(3, 1))


results = gt.LevenbergMarquardtOptimizer(graph, initial).optimize() 
marginals = gt.Marginals(graph, results) 
for i in range(1, 6): 
    gtsam_plot.plot_pose2(0, results.atPose2(i), 0.5, marginals.marginalCovariance(i))
gtsam_plot.plot_point2(0, results.atPoint2(anchor_key), 0.5, marginals.marginalCovariance(anchor_key))
plt.axis('equal')
plt.savefig('gtsam_loop.png')
print("RESULTS:", results, sep="\n")

