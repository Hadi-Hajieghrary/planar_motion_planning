#include <SFML/Graphics.hpp> // For visualization
#include "workspace.hpp"      // Workspace definition
#include "rrt.hpp"            // RRT-based path planning
#include "robot.hpp"          // Robot definition and kinematics

int main() {
    // Initialize robot with specific dimensions and joint bounds
    BoundedValue theta1(-M_PI, M_PI, M_PI / 4.0);
    BoundedValue theta2(-M_PI, M_PI, -M_PI / 4.0);
    BoundedValue theta3(-M_PI, M_PI, M_PI / 4.0);
    Robot robot(0.3, 1.0, 1.0, 0.5, 0.1, theta1, theta2, theta3);

    Workspace workspace(2.0, 2.0, robot);

    // Add obstacles to the workspace
    workspace.addObstacle(Box({{1.5, 3.0}, 0.0}, 1.0, 1.0));
    workspace.addObstacle(Box({{1.0, 0.5}, 0.0}, 1.0, 1.0));

    // Define the pick subject (object to pick up)
    Box pick_subject({{1.0, 1.5}, 0.0}, 0.75, 0.75);
    workspace.setSubject(pick_subject);

    // Plan the trajectory to the pick location
    RRT pick_rrt(workspace, 0.01, 0.05, 20000);
    std::vector<std::array<double, 3>> pick_trajectory;
    if (!pick_rrt.computeTrajectory(pick_trajectory, false)) {
        std::cerr << "Failed to find pick trajectory!\n";
        return -1;
    }

    // Define the place target (location to place the object)
    Box place_target({{-1.0, 1.5}, 0.0}, 0.75, 0.75);
    workspace.setSubject(place_target);

    // Plan the trajectory to the place location
    RRT place_rrt(workspace, 0.01, 0.05, 20000);
    std::vector<std::array<double, 3>> place_trajectory;
    if (!place_rrt.computeTrajectory(place_trajectory, true)) {
        std::cerr << "Failed to find place trajectory!\n";
        return -1;
    }

    // Visualization parameters
    constexpr double WINDOW_WIDTH = 800;
    constexpr double WINDOW_HEIGHT = 800;
    constexpr double SCALE_X = WINDOW_WIDTH / 10.0;
    constexpr double SCALE_Y = WINDOW_HEIGHT / 10.0;

    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Robot Animation");

    int frameCounter = 0; // Counter for saving frames

    // Animate the pick trajectory
    animateTrajectory(window, workspace, robot, pick_trajectory, pick_subject, {SCALE_X, SCALE_Y}, sf::Color::Blue, frameCounter);

    // Animate the place trajectory
    animateTrajectory(window, workspace, robot, place_trajectory, pick_subject, {SCALE_X, SCALE_Y}, sf::Color::Green, frameCounter);

    return 0;
}
