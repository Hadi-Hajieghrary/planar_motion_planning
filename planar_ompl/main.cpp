#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/config.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <cmath>

// Structure to represent the R3Robot model
// This represents a 3-link planar robotic manipulator with fixed link lengths and a configurable width.
struct R3Robot
{
    const double l1{1.0}; // Length of the first link
    const double l2{1.0}; // Length of the second link
    const double l3{1.0}; // Length of the third link
    const double width{0.1}; // Width of the links, used for collision checks

    // Computes the forward kinematics for the robot given joint angles.
    // Calculates the (x, y) position of the end-effector.
    void forwardKinematics(const std::vector<double> &angles, double &x, double &y) const
    {
        const double& theta1 = angles[0];
        const double& theta2 = angles[1];
        const double& theta3 = angles[2];

        // Compute the x and y positions of the end-effector by summing contributions from each link.
        x = l1 * cos(theta1) + l2 * cos(theta1 + theta2) + l3 * cos(theta1 + theta2 + theta3);
        y = l1 * sin(theta1) + l2 * sin(theta1 + theta2) + l3 * sin(theta1 + theta2 + theta3);
    }

    // Computes forward kinematics for all joint positions, including the base.
    // Outputs a list of (x, y) positions for each joint and the end-effector.
    void forwardKinematicsAll(const std::vector<double> &angles, std::vector<std::pair<double, double>> &positions) const
    {
        const double& theta1 = angles[0];
        const double& theta2 = angles[1];
        const double& theta3 = angles[2];

        // Calculate and store positions of the base, joints, and end-effector.
        positions = {
            {0.0, 0.0}, // Base position
            {l1 * cos(theta1), l1 * sin(theta1)}, // Position of the first joint
            {l1 * cos(theta1) + l2 * cos(theta1 + theta2), l1 * sin(theta1) + l2 * sin(theta1 + theta2)}, // Position of the second joint
            {l1 * cos(theta1) + l2 * cos(theta1 + theta2) + l3 * cos(theta1 + theta2 + theta3),
             l1 * sin(theta1) + l2 * sin(theta1 + theta2) + l3 * sin(theta1 + theta2 + theta3)} // End-effector position
        };
    }

    // Checks if a single link of the robot collides with a rectangular obstacle.
    bool isLinkInCollision(const std::pair<double, double> &start, const std::pair<double, double> &end, const std::array<double, 4> &obs) const
    {
        // Extract start and end coordinates of the link
        const double& x1 = start.first;
        const double& y1 = start.second;
        const double& x2 = end.first;
        const double& y2 = end.second;

        // Expand obstacle boundaries to account for the link's width
        const double expanded_x_min = obs[0] - width / 2;
        const double expanded_y_min = obs[1] - width / 2;
        const double expanded_x_max = obs[2] + width / 2;
        const double expanded_y_max = obs[3] + width / 2;

        // Check if either endpoint of the link is inside the expanded obstacle
        if ((x1 >= expanded_x_min && x1 <= expanded_x_max && y1 >= expanded_y_min && y1 <= expanded_y_max) ||
            (x2 >= expanded_x_min && x2 <= expanded_x_max && y2 >= expanded_y_min && y2 <= expanded_y_max))
        {
            return true;
        }

        // Check if the bounding box of the link overlaps with the expanded obstacle
        const double link_min_x = std::min(x1, x2);
        const double link_max_x = std::max(x1, x2);
        const double link_min_y = std::min(y1, y2);
        const double link_max_y = std::max(y1, y2);

        if (link_max_x >= expanded_x_min && link_min_x <= expanded_x_max &&
            link_max_y >= expanded_y_min && link_min_y <= expanded_y_max)
        {
            return true;
        }

        return false; // No collision detected
    }
};

// Validates if a given robot state is collision-free
bool isStateValid(const ompl::base::State *state, const R3Robot &robot)
{
    // Extract joint angles from the OMPL state
    const auto *realState = state->as<ompl::base::RealVectorStateSpace::StateType>();
    std::vector<double> angles = {realState->values[0], realState->values[1], realState->values[2]};

    // Compute positions of all links and joints
    std::vector<std::pair<double, double>> link_positions;
    robot.forwardKinematicsAll(angles, link_positions);

    // Define a list of rectangular obstacles (x_min, y_min, x_max, y_max)
    std::vector<std::array<double, 4>> obstacles = {
        {1.0, 1.0, 2.0, 2.0},
        {3.0, 3.0, 4.0, 4.0},
        {2.0, 0.5, 3.0, 1.5},
        {1.5, 2.5, 2.5, 3.5}
    };

    // Check each link for collisions with any obstacle
    for (size_t i = 0; i < link_positions.size() - 1; ++i)
    {
        for (const auto &obs : obstacles)
        {
            if (robot.isLinkInCollision(link_positions[i], link_positions[i + 1], obs))
            {
                return false; // Collision detected
            }
        }
    }
    return true; // No collisions detected
}

int main()
{
    // Create a 3D state space to represent joint angles
    auto space = std::make_shared<ompl::base::RealVectorStateSpace>(3);

    // Set bounds for each joint angle (-pi to pi)
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(-M_PI);
    bounds.setHigh(M_PI);
    space->setBounds(bounds);

    // Setup the SimpleSetup object with the defined state space
    ompl::geometric::SimpleSetup ss(space);
    R3Robot robot; // Instantiate the robot

    // Set state validity checker to ensure collision-free states
    ss.setStateValidityChecker([&robot](const ompl::base::State *state) { return isStateValid(state, robot); });

    // Define start and goal states in joint space
    ompl::base::ScopedState<> start(space), goal(space);
    start = {M_PI / 8, -M_PI / 8, -M_PI / 6};
    goal = {M_PI / 2, M_PI / 4, -M_PI / 4};
    ss.setStartAndGoalStates(start, goal);

    // Attempt to solve the motion planning problem
    if (ss.solve(1.0))
    {
        std::cout << "Found solution:\n";
        ss.simplifySolution(); // Simplify the path
        ss.getSolutionPath().interpolate(100); // Interpolate the solution path for smoothness
        ss.getSolutionPath().print(std::cout); // Print the solution path to console

        // Write the solution path to a CSV file
        std::ofstream outFile("../../src/planar_ompl/data/path.csv");
        if (outFile)
        {
            for (const auto *state : ss.getSolutionPath().getStates())
            {
                const auto *realState = state->as<ompl::base::RealVectorStateSpace::StateType>();
                std::vector<double> angles = {realState->values[0], realState->values[1], realState->values[2]};
                std::vector<std::pair<double, double>> positions;
                robot.forwardKinematicsAll(angles, positions);

                // Write each joint and end-effector position to the file
                for (const auto &pos : positions)
                {
                    outFile << pos.first << "," << pos.second << ",";
                }
                outFile << "\n";
            }
        }
        else
        {
            std::cerr << "Unable to open file to write solution path." << std::endl;
        }
    }
    else
    {
        std::cout << "No solution found" << std::endl;
    }

    return 0; // Exit the program
}
