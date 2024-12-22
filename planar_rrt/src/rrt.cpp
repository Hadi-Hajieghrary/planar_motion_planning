#include "rrt.hpp" // Include header file for RRT class definition
#include <algorithm>  // For std::min_element
#include <cmath>      // For mathematical operations
#include <iostream>   // For debugging and error outputs

// Constructor: Initialize RRT planner with workspace, thresholds, and limits
RRT::RRT(Workspace& workspace, double goal_threshold, double max_step_size, int max_iterations)
    : workspace_(workspace),
      goal_threshold_(goal_threshold),
      max_step_size_(max_step_size),
      max_iterations_(max_iterations) {
    // Retrieve the joint limits of the robot from the workspace
    joint_limits_ = workspace.getRobot().getJointValues();

    // Compute the goal joint values using inverse kinematics
    if (!workspace_.getRobot().inverseKinematics(workspace_.getSubject().getPose(), goal_joint_values_)) {
        std::cerr << "Error: Inverse kinematics failed for the subject's pose." << std::endl;
        throw std::runtime_error("Unable to compute goal joint values.");
    }

    // Print debug information for the goal pose and joint values
    std::cout << "Goal Pose: " 
              << "X: " << workspace_.getSubject().getPose().position.x << ",\t" 
              << "Y: " << workspace_.getSubject().getPose().position.y << ",\t" 
              << "Theta: " << workspace_.getSubject().getPose().orientation << std::endl;
    std::cout << "Goal Joint Values: " 
              << goal_joint_values_[0] << ",\t" 
              << goal_joint_values_[1] << ",\t" 
              << goal_joint_values_[2] << std::endl;
}

// Generate a random configuration within the robot's joint limits
std::array<double, 3> RRT::randomConfiguration() {
    auto config = std::array<double, 3>{
        joint_limits_[0].getRandomValue(), // Random value for joint 1
        joint_limits_[1].getRandomValue(), // Random value for joint 2
        joint_limits_[2].getRandomValue()  // Random value for joint 3
    };
    if (DEBUG_MODE) {
        std::cout << "Random Configuration: " << config[0] << ", " << config[1] << ", " << config[2] << std::endl;
    }
    return config;
}

// Find the nearest node in the tree to a given sample configuration
std::shared_ptr<Node> RRT::nearestNode(const std::array<double, 3>& sample) {
    return *std::min_element(nodes_.begin(), nodes_.end(), [&sample](const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) {
        return distance(a->theta_values, sample) < distance(b->theta_values, sample);
    });
}

// Steer the tree towards a sample configuration with a maximum step size
std::array<double, 3> RRT::steer(const std::array<double, 3>& from, const std::array<double, 3>& to) {
    double dist = distance(from, to); // Compute distance between configurations
    if (dist <= max_step_size_) {
        return to; // Return target configuration if within step size
    }

    // Compute step in the direction of the target configuration
    std::array<double, 3> direction = {
        (to[0] - from[0]) / dist,
        (to[1] - from[1]) / dist,
        (to[2] - from[2]) / dist
    };

    // Compute the new configuration by stepping towards the target
    auto new_config = std::array<double, 3>{
        from[0] + direction[0] * max_step_size_,
        from[1] + direction[1] * max_step_size_,
        from[2] + direction[2] * max_step_size_
    };

    if (DEBUG_MODE) {
        std::cout << "Steered Configuration: " << new_config[0] << ", " << new_config[1] << ", " << new_config[2] << std::endl;
    }
    return new_config;
}

// Check if a configuration is collision-free
bool RRT::isCollisionFree(const std::array<double, 3>& configuration, bool grasped) {
    workspace_.getRobot().setJointValues(configuration); // Set robot to the given configuration
    bool collision_free = !workspace_.isThereCollision(grasped); // Check for collisions
    if (DEBUG_MODE) {
        std::cout << "Configuration " << (collision_free ? "is collision-free" : "is in collision") 
                  << ": " << configuration[0] << ", " << configuration[1] << ", " << configuration[2] << std::endl;
    }
    return collision_free;
}

// Check if a configuration is within the goal threshold
bool RRT::isGoalReached(const std::array<double, 3>& configuration) {
    bool goal_reached = distance(configuration, goal_joint_values_) <= goal_threshold_;
    if (goal_reached) {
        std::cout << "Goal reached with configuration: " 
                  << configuration[0] << ", " 
                  << configuration[1] << ", " 
                  << configuration[2] << std::endl;
    }
    return goal_reached;
}

// Compute a trajectory to the goal using the RRT algorithm
bool RRT::computeTrajectory(std::vector<std::array<double, 3>>& trajectory, bool grasped) {
    trajectory.clear(); // Clear any existing trajectory

    // Initialize the tree with the start configuration
    auto start_joint_values = workspace_.getRobot().getJointValues();
    auto start_node = std::make_shared<Node>(std::array<double, 3>{
        start_joint_values[0].getValue(),
        start_joint_values[1].getValue(),
        start_joint_values[2].getValue()
    });
    nodes_.push_back(start_node);

    // Iterate to grow the RRT tree
    for (int iter = 0; iter < max_iterations_; ++iter) {
        // Generate random configuration (goal biasing every 10 iterations)
        std::array<double, 3> theta_rand = (iter % 10 == 0) ? goal_joint_values_ : randomConfiguration();

        // Find the nearest node to the random sample
        auto nearest = nearestNode(theta_rand);

        // Steer towards the random configuration
        std::array<double, 3> theta_new = steer(nearest->theta_values, theta_rand);

        // Check if the new configuration is collision-free
        if (isCollisionFree(theta_new, grasped)) {
            // Add the new configuration as a node in the tree
            auto new_node = std::make_shared<Node>(theta_new, nearest);
            nodes_.push_back(new_node);

            // Check if the goal is reached
            if (isGoalReached(theta_new)) {
                // Reconstruct the trajectory by backtracking from the goal node
                auto current = new_node;
                while (current) {
                    trajectory.push_back(current->theta_values);
                    current = current->parent;
                }
                std::reverse(trajectory.begin(), trajectory.end()); // Reverse to get the correct order
                std::cout << "Trajectory found with " << nodes_.size() << " nodes." << std::endl;
                return true;
            }
        }
    }

    // If the goal is not reached, print debug info and return false
    std::cout << "Goal not reached. Tree size: " << nodes_.size() << "\n";
    return false;
}
