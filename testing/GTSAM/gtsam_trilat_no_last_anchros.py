import gtsam as gt 
import gtsam.utils.plot as gtsam_plot 
import matplotlib.pyplot as plt 
from math import pi 
import random 

ANCHOR_POS_NOISE = gt.noiseModel.Diagonal.Sigmas([0.02, 0.02]) # uncertainty in placing anchors 
ODOMETRY_NOISE = gt.noiseModel.Diagonal.Sigmas([0.3, 0.3, 0.15]) 
RANGE_NOISE = gt.noiseModel.Isotropic.Sigma(1, 0.1) # precise 1D measurement 

# Symbols for landmarks and states
x1 = gt.symbol('x', 1) 
x2 = gt.symbol('x', 2) 
x3 = gt.symbol('x', 3) 
x4 = gt.symbol('x', 4) 
x5 = gt.symbol('x', 5) 

z1 = gt.symbol('l', 1)
z2 = gt.symbol('l', 2)
z3 = gt.symbol('l', 3)
z4 = gt.symbol('l', 4)



# Defining anchor positions and odometry movements with my reference graph 
anchor_prior_pos = [(0, -1), (0, 1), (-1, 0), (3,3)]
odom_mvts = [(1, 0), (1,1), (1,1), (1,0)]

# Graph 
graph = gt.NonlinearFactorGraph() 

# Adding anchor locations and measurements
for z, (x, y) in zip([z1, z2, z3, z4], anchor_prior_pos): 
    graph.add(gt.PriorFactorPoint2(z, gt.Point2(x, y), ANCHOR_POS_NOISE))

graph.add(gt.RangeFactor2D(x1, z1, 1.1, RANGE_NOISE))
graph.add(gt.RangeFactor2D(x1, z2, 1, RANGE_NOISE))
graph.add(gt.RangeFactor2D(x1, z3, 0.95, RANGE_NOISE))

graph.add(gt.RangeFactor2D(x4, z4, 1.05, RANGE_NOISE))


# Odometry 
for (s1, s2), (x, y) in zip([(x1, x2), (x2,x3), (x3,x4), (x4,x5)], odom_mvts): 
    graph.add(gt.BetweenFactorPose2(s1, s2, gt.Pose2(x, y, 0), ODOMETRY_NOISE))

## Solving 
print("GRAPH:", graph, sep="\n")

initial = gt.Values()
for z, (x, y) in zip([z1, z2, z3, z4], anchor_prior_pos): 
    initial.insert(z, gt.Point2(x, y))
initial.insert(x1, gt.Pose2(-0.5, 0.1, 0))
initial.insert(x2, gt.Pose2(1.1, -0.1, 0))
initial.insert(x3, gt.Pose2(2.2, 1.2, 0))
initial.insert(x4, gt.Pose2(2.7, 1.9, 0))
initial.insert(x5, gt.Pose2(4.1, 2.2, 0))

results = gt.LevenbergMarquardtOptimizer(graph, initial).optimize() 
marginals = gt.Marginals(graph, results) 
for x in [x1, x2, x3, x4, x5]: 
    gtsam_plot.plot_pose2(0, results.atPose2(x), 0.5, marginals.marginalCovariance(x))
for z in [z1, z2, z3, z4]: 
    gtsam_plot.plot_point2(0, results.atPoint2(z), 0.5, marginals.marginalCovariance(z))
plt.axis('equal')
plt.savefig('gtsam_loop.png')
print("RESULTS:", results, sep="\n")
