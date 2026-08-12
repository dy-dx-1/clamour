import gtsam as gt 
import gtsam.utils.plot as gtsam_plot 
import matplotlib.pyplot as plt 
from random import gauss

### Noise
ANCHOR_POS_NOISE = gt.noiseModel.Diagonal.Sigmas([0.02, 0.02]) # uncertainty in placing anchors 
ODOMETRY_NOISE = gt.noiseModel.Diagonal.Sigmas([0.3, 0.3, 0.15]) 
RANGE_NOISE = gt.noiseModel.Isotropic.Sigma(1, 0.1) # precise 1D measurement 

# Defining anchor positions and odometry movements for this example
# {state: ( odom_mvt , {'anchor_id': int, 'anchor_positions': list of tuples, 'measurements': float} )}
# odom_mvt is the one performed towards the state in question
motion_and_measurements = {1: (None, {'a_ids':[1, 2, 3], 'a_pos':[(0, -1), (0,1), (-1,0)], 'zs': [1+gauss(0, 0.1) for _ in range(3)]} ),
                           2: ((1,0), None),
                           3: ((1,1), None),
                           4: ((1,1), {'a_ids':[4], 'a_pos':[(3,3)], 'zs':[1+gauss(0, 0.1)]}),
                           5: ((1,0), {'a_ids':[5, 6, 7], 'a_pos':[(4, 3), (5, 2), (4,1)], 'zs': [1+gauss(0, 0.1) for _ in range(3)]})}


### Starting ISAM 
isam = gt.ISAM2() 
results = None # update throughout loop, allows us to fetch previous best estimate
### Iterating and updating graph according to how many states to consider
for state_id, (motion, measurements) in list(motion_and_measurements.items())[:]:  # index allows to play around with simulation 
    graph = gt.NonlinearFactorGraph()
    initial = gt.Values() 

    x = gt.symbol('x', state_id) 
    # If there's an odometry movement that brought us here, incorporate it 
    if motion: 
        x_prev = gt.symbol('x', state_id-1)
        mvt = gt.Pose2(motion[0], motion[1], 0)
        graph.add(gt.BetweenFactorPose2(x_prev, x, mvt, ODOMETRY_NOISE))

        # Set initial value for the state to guess with odometry 
        previous_pose = results.atPose2(x_prev) 
        initial.insert(x, previous_pose.compose(mvt))
    else: # ONLY FOR THE FIRST STATE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 
        # if no motion, this is the first state, just add an initial belief for it 
        # needs a prior to properly initialize solver, else undertermined error on first run 
        initial.insert(x, gt.Pose2(0,0,0))
        graph.add(gt.PriorFactorPose2(x, gt.Pose2(0,0,0), gt.noiseModel.Diagonal.Sigmas([0.1,0.1,0.1])))
    # If there's measurements here, add them 
    if measurements: 
        for a_id, a_pos, z in zip(measurements['a_ids'], measurements['a_pos'], measurements['zs']): 
            l = gt.symbol('l', a_id)
            graph.add(gt.PriorFactorPoint2(l, gt.Point2(a_pos[0], a_pos[1]), ANCHOR_POS_NOISE))
            graph.add(gt.RangeFactor2D(x, l, z, RANGE_NOISE))

            initial.insert(l, gt.Point2(a_pos[0], a_pos[1])) # initial value for anchor 
    # Finally, update ISAM and recalculate 
    isam.update(graph, initial) 
    results = isam.calculateEstimate() 

### plot estimates 
# Recuperating marginal posterior density for each of the states 
for key in results.keys(): 
    cov_matrix = isam.marginalCovariance(key)  # directly querying instead of batch Marginals like with regular FG 
    print(f"Covariance matrix for state {key}", cov_matrix, sep="\n")
    if gt.Symbol(key).chr() == ord('x'): 
        gtsam_plot.plot_pose2(0, results.atPose2(key), 0.5, cov_matrix)
    else:
        gtsam_plot.plot_point2(0, results.atPoint2(key), 0.5, cov_matrix)

plt.axis('equal')
plt.savefig('isam_trilat.png')
print("RESULTS:", results, sep="\n")
