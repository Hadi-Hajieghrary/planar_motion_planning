#ifndef __RRT_HPP__
#define __RRT_HPP__

#include "utility.hpp"
#include "workspace.hpp"
#include <array>
#include <vector>
#include <memory>
#include <random>

static bool DEBUG_MODE = false;

// Node structure representing a configuration in the RRT
struct Node {
    std::array<double, 3> theta_values;
    std::shared_ptr<Node> parent;

    Node(const std::array<double, 3>& theta_values, std::shared_ptr<Node> parent = nullptr)
        : theta_values(theta_values), parent(parent) {}
};

class RRT {
private:
    Workspace& workspace_;
    std::array<BoundedValue, 3> joint_limits_;
    std::array<double, 3> goal_joint_values_;
    std::vector<std::shared_ptr<Node>> nodes_;
    double goal_threshold_;
    double max_step_size_;
    int max_iterations_;

    // Generate a random configuration within joint limits
    std::array<double, 3> randomConfiguration();

    // Find the nearest node in the tree
    std::shared_ptr<Node> nearestNode(const std::array<double, 3>& sample);

    // Steer the tree towards the sample
    std::array<double, 3> steer(const std::array<double, 3>& from, const std::array<double, 3>& to);

    // Check for collision-free configuration
    bool isCollisionFree(const std::array<double, 3>& configuration, bool grasped = false);

    // Check if the configuration is within the goal threshold
    bool isGoalReached(const std::array<double, 3>& configuration);

public:
    RRT(Workspace& workspace, double goal_threshold = 0.2, double max_step_size = 0.01, int max_iterations = 10000);

    // Compute the trajectory to the goal
    bool computeTrajectory(std::vector<std::array<double, 3>>& trajectory, bool grasped = false);
};

#endif
