import gtsam as gt 
import numpy as np 
import matplotlib.pyplot as plt 
import gtsam.utils.plot as gtsam_plot 

# Empty factor graph. This will represent the joint probability distribution P(X|Z) over the ENTIRE trajectory 
graph = gt.NonlinearFactorGraph() 

# Defining the prior factor f_1(x1) 
prior_mean = gt.Pose2(0.0, 0.0, 0.0)  # State is 2D + orientation 
prior_noise = gt.noiseModel.Diagonal.Sigmas([0.3, 0.3, 0.1])  # Std devs of state (sigma equivalent of P) 
# Adding the prior to the graph 
graph.add(gt.PriorFactorPose2(1, prior_mean, prior_noise)) # factor connected to state 1, mean and noise of the probability distribution 

# Defining odometry factors, both have the same mean and noise in this case 
o_mean = gt.Pose2(2.0, 0.0, 0.0)  
o_noise = gt.noiseModel.Diagonal.Sigmas([0.2, 0.2, 0.1]) 
# Adding the factors to the graph
graph.add(gt.BetweenFactorPose2(1, 2, o_mean, o_noise))  # Adding factor between vars 1 and 2 with odom mean and noise 
graph.add(gt.BetweenFactorPose2(2, 3, o_mean, o_noise))

# Adding a custom GPS factor at pose 2 
def gps_factor_error(mx, my, this: gt.CustomFactor, values: gt.Values, H: list[np.ndarray]):
    """
    Error evaluation function for custom GPS-like measurement
    """
    key = this.keys()[0] # Getting the key of the state that was called with this factor 
    q = values.atPose2(key) # Values of the associated belief 

    if H is not None: 
        R = q.rotation()
        H[0] = np.array([
                    [R.c(), -R.s(), 0.0],
                    [R.s(),  R.c(), 0.0]
                ])
        
    # Return the measurement error vector (dim 2)
    return np.array([q.x() - mx, q.y() - my])      
from functools import partial
mx, my = 2, 0
gps_noise = gt.noiseModel.Diagonal.Sigmas([0.1, 0.1])
graph.add(gt.CustomFactor(gps_noise, [2], partial(gps_factor_error, mx, my)))


print("CONSTRUCTED GRAPH: ", graph, sep="\n") 

# Defining initial values for states 
initial_states = gt.Values() 
initial_states.insert(1, gt.Pose2(0.5, 0.0, 0.2))
initial_states.insert(2, gt.Pose2(2.3, 0.1, -0.2))
initial_states.insert(3, gt.Pose2(4.1, 0.1, 0.1))
# Optimizing 
results = gt.LevenbergMarquardtOptimizer(graph, initial_states).optimize() 
print("RESULTS:", results, sep="\n")

# Recuperating marginal posterior density for each of the states 
marginals = gt.Marginals(graph, results) 
for key in range(1, 4): # states 1, 2, 3
    cov_matrix = marginals.marginalCovariance(key) 
    print(f"Covariance matrix for state {key}", cov_matrix, sep="\n")
    gtsam_plot.plot_pose2(0, results.atPose2(key), 0.5, marginals.marginalCovariance(key))

plt.axis('equal')
plt.savefig('gtsam_intro.png')
print("RESULTS:", results, sep="\n")
