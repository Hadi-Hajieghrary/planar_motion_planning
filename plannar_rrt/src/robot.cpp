#include "robot.hpp" // Include header for Robot class definition
#include <iostream>   // For debugging outputs

// Constructor: Initialize the robot's physical dimensions and joint bounds
Robot::Robot(double link_0_length, double link_1_length, double link_2_length, double link_3_length, double link_width, const BoundedValue& theta_1, const BoundedValue& theta_2, const BoundedValue& theta_3)
: link_0_length_{link_0_length}, link_1_length_{link_1_length}, link_2_length_{link_2_length}, link_3_length_{link_3_length}, 
  link_width_{link_width},
  theta_1_{theta_1}, theta_2_{theta_2}, theta_3_{theta_3} {

    // Initialize the base link's pose
    link_0_.position.x = 0.0;
    link_0_.position.y = 0.0;
    link_0_.orientation = M_PI / 2.0;

    // Set the initial joint values for the robot
    this->setJointValues(theta_1.getValue(), theta_2.getValue(), theta_3.getValue());

    // Initialize collision objects for each link of the robot
    this->collision_objects_.push_back(Box({0.0, 0.0, M_PI / 2.0}, this->link_width_, this->link_0_length_));
    this->collision_objects_.push_back(Box({this->link_1_.position.x, this->link_1_.position.y, this->link_1_.orientation}, this->link_width_, this->link_1_length_));
    this->collision_objects_.push_back(Box({this->link_2_.position.x, this->link_2_.position.y, this->link_2_.orientation}, this->link_width_, this->link_2_length_));
    this->collision_objects_.push_back(Box({this->link_3_.position.x, this->link_3_.position.y, this->link_3_.orientation}, this->link_width_, this->link_3_length_));
}

// Move assignment operator: Transfers ownership of resources
Robot& Robot::operator=(Robot&& other) {
    if (this != &other) {
        const_cast<double&>(link_0_length_) = std::move(other.link_0_length_);
        const_cast<double&>(link_1_length_) = std::move(other.link_1_length_);
        const_cast<double&>(link_2_length_) = std::move(other.link_2_length_);
        const_cast<double&>(link_3_length_) = std::move(other.link_3_length_);
        const_cast<double&>(link_width_) = std::move(other.link_width_);
        theta_1_ = std::move(other.theta_1_);
        theta_2_ = std::move(other.theta_2_);
        theta_3_ = std::move(other.theta_3_);
        link_0_ = std::move(other.link_0_);
        link_1_ = std::move(other.link_1_);
        link_2_ = std::move(other.link_2_);
        link_3_ = std::move(other.link_3_);
        link_ee_ = std::move(other.link_ee_);
        collision_objects_ = std::move(other.collision_objects_);
    }
    return *this;
}

// Set the robot's joint values and update link positions
void Robot::setJointValues(const double theta1, const double theta2, const double theta3) {
    // Ensure joint values stay within their respective bounds
    this->theta_1_.setValue(theta1);
    this->theta_2_.setValue(theta2);
    this->theta_3_.setValue(theta3);

    // Compute positions and orientations of each link based on joint values
    link_1_.position.x = 0.0;
    link_1_.position.y = this->link_0_length_;
    link_1_.orientation = theta_1_.getValue();

    link_2_.position.x = link_1_.position.x + this->link_1_length_ * cos(link_1_.orientation);
    link_2_.position.y = link_1_.position.y + this->link_1_length_ * sin(link_1_.orientation);
    link_2_.orientation = link_1_.orientation - theta_2_.getValue();

    link_3_.position.x = link_2_.position.x + this->link_2_length_ * cos(link_2_.orientation);
    link_3_.position.y = link_2_.position.y + this->link_2_length_ * sin(link_2_.orientation);
    link_3_.orientation = link_2_.orientation + theta_3_.getValue();

    link_ee_.position.x = link_3_.position.x + this->link_3_length_ * cos(link_3_.orientation);
    link_ee_.position.y = link_3_.position.y + this->link_3_length_ * sin(link_3_.orientation);
    link_ee_.orientation = link_3_.orientation;
}

// Overload: Set joint values using an array
void Robot::setJointValues(const std::array<double, 3> thetas) {
    setJointValues(thetas[0], thetas[1], thetas[2]);
}

// Retrieve the pose of the end-effector
const Pose2D Robot::getEEPose() const {
    return this->link_ee_;
}

// Retrieve the pose of a specific link
const Pose2D Robot::getLinkPose(const Robot::Link& link) const {
    switch (link) {
    case Robot::Link::Link_1:
        return this->link_1_;
    case Robot::Link::Link_2:
        return this->link_2_;
    case Robot::Link::Link_3:
        return this->link_3_;
    case Robot::Link::Link_ee:
        return this->link_ee_;
    default:
        throw std::out_of_range("Invalid link index! Robot has 3 links and an end-effector.");
    }
}

// Retrieve the end-effector pose for specific joint values
const Pose2D Robot::getEEPose(const double theta1, const double theta2, const double theta3) {
    setJointValues(theta1, theta2, theta3);
    return this->link_ee_;
}

// Populate collision objects based on the current robot state
void Robot::getRobotCollisionObjects(std::vector<Box>& collision_objects) const {
    collision_objects.clear();
    collision_objects.push_back(Box({0.0, 0.0, M_PI / 2.0}, this->link_width_, this->link_0_length_));
    collision_objects.push_back(Box({this->link_1_.position.x, this->link_1_.position.y, this->link_1_.orientation}, this->link_width_, this->link_1_length_));
    collision_objects.push_back(Box({this->link_2_.position.x, this->link_2_.position.y, this->link_2_.orientation}, this->link_width_, this->link_2_length_));
    collision_objects.push_back(Box({this->link_3_.position.x, this->link_3_.position.y, this->link_3_.orientation}, this->link_width_, this->link_3_length_));
}

// Include an additional subject box as part of the collision objects
void Robot::getRobotCollisionObjects(Box& subject, std::vector<Box>& collision_objects) const {
    getRobotCollisionObjects(collision_objects); // Retrieve existing collision objects
    subject.setPose({this->link_ee_.position.x, this->link_ee_.position.y, this->link_ee_.orientation});
    collision_objects.push_back(subject);
}

// Compute the inverse kinematics for a given end-effector pose
bool Robot::inverseKinematics(Pose2D ee_pose, std::array<double, 3>& joint_values) {
    joint_values = {0.0, 0.0, 0.0}; // Initialize joint values
    ee_pose.position.y -= this->link_0_length_; // Adjust for base link length

    // Compute wrist position
    double x_wrist = ee_pose.position.x - link_3_length_ * cos(ee_pose.orientation);
    double y_wrist = ee_pose.position.y - link_3_length_ * sin(ee_pose.orientation);

    double d1 = std::sqrt(x_wrist * x_wrist + y_wrist * y_wrist); // Distance to wrist
    double d2 = link_1_length_ + link_2_length_;
    if (d1 > d2 + 1e-9) {
        return false; // Pose is unreachable
    }

    double cTheta = (x_wrist * x_wrist + y_wrist * y_wrist - link_1_length_ * link_1_length_ - link_2_length_ * link_2_length_) / (2.0 * link_1_length_ * link_2_length_);
    cTheta = std::max(-1.0, std::min(1.0, cTheta)); // Clamp cosine to [-1, 1]
    double theta2_options[2] = {atan2(sqrt(1 - cTheta * cTheta), cTheta), atan2(-sqrt(1 - cTheta * cTheta), cTheta)};
    for (int i = 0; i < 2; ++i) {
        double theta2 = theta2_options[i];
        double k1 = link_1_length_ + link_2_length_ * cos(theta2);
        double k2 = -link_2_length_ * sin(theta2);
        double theta1 = atan2(y_wrist, x_wrist) - atan2(k2, k1);

        double theta3 = ee_pose.orientation - theta1 + theta2;
        theta1 = atan2(sin(theta1), cos(theta1));
        theta2 = atan2(sin(theta2), cos(theta2));
        theta3 = atan2(sin(theta3), cos(theta3));

        if (theta_1_.isWithinBounds(theta1) && theta_2_.isWithinBounds(theta2) && theta_3_.isWithinBounds(theta3)) {
            joint_values = {theta1, theta2, theta3};
            return true;
        }
    }
    return false; // No valid solutions
}

// Render the robot and its components on a visualization window
void Robot::Draw(sf::RenderWindow& window, const std::pair<double, double> scale) const {
    const double& scale_x = scale.first;
    const double& scale_y = scale.second;
    const double unified_scale = std::min(scale_x, scale_y); // Uniform scaling
    const sf::Vector2f window_center(window.getSize().x / 2.0f, window.getSize().y / 2.0f);

    // Helper function: Create a rectangle for a robot link
    auto createRectangle = [&](const Pose2D& link_pose, double width, double height) -> sf::RectangleShape {
        sf::RectangleShape rectangle(sf::Vector2f(width * unified_scale, height * unified_scale));
        rectangle.setOrigin(0, height * unified_scale / 2); // Origin at mid-left
        rectangle.setPosition(
            window_center.x + link_pose.position.x * unified_scale,
            window_center.y - link_pose.position.y * unified_scale
        );
        rectangle.setRotation(-link_pose.orientation * 180.0 / M_PI); // Convert radians to degrees
        rectangle.setFillColor(sf::Color::Green);
        return rectangle;
    };

    // Helper function: Create a circle for a robot joint
    auto createCircle = [&](const Point2D& position, double radius) -> sf::CircleShape {
        sf::CircleShape circle(radius * unified_scale);
        circle.setOrigin(radius * unified_scale, radius * unified_scale); // Center origin
        circle.setPosition(
            window_center.x + position.x * unified_scale,
            window_center.y - position.y * unified_scale
        );
        circle.setFillColor(sf::Color::Red);
        return circle;
    };

    // Draw robot links
    sf::RectangleShape link0 = createRectangle(link_0_, link_0_length_, link_width_);
    sf::RectangleShape link1 = createRectangle(link_1_, link_1_length_, link_width_);
    sf::RectangleShape link2 = createRectangle(link_2_, link_2_length_, link_width_);
    sf::RectangleShape link3 = createRectangle(link_3_, link_3_length_, link_width_);

    // Draw robot joints
    sf::CircleShape joint1 = createCircle(link_1_.position, link_width_ / 2.0);
    sf::CircleShape joint2 = createCircle(link_2_.position, link_width_ / 2.0);
    sf::CircleShape joint3 = createCircle(link_3_.position, link_width_ / 2.0);

    // Draw end-effector
    sf::CircleShape ee_circle = createCircle(link_ee_.position, link_width_ / 2.0);
    ee_circle.setFillColor(sf::Color::Blue);

    // Render all shapes on the window
    window.draw(link0);
    window.draw(link1);
    window.draw(link2);
    window.draw(link3);
    window.draw(joint1);
    window.draw(joint2);
    window.draw(joint3);
    window.draw(ee_circle);
}
