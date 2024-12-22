#ifndef __ROBOT_HPP__
#define __ROBOT_HPP__


#include <vector>
#include <array>
#include <cmath>
#include <stdexcept>

#include "utility.hpp"
#include <SFML/Graphics.hpp>


/*Robot
* Class representing the Robot in 2D
*/
class Robot {

private:

    const double link_0_length_, link_1_length_, link_2_length_, link_3_length_; 
    const double link_width_;

    BoundedValue theta_1_, theta_2_, theta_3_;

    Pose2D link_0_, link_1_, link_2_, link_3_, link_ee_;

    std::vector<Box> collision_objects_;

public:

    enum class Link{Link_1, Link_2, Link_3, Link_ee};

    // Constructor to initialize link lengths/width and initial joint values/bounds
    Robot(double link_0_length, double link_1_length, double link_2_length, double link_3_length, 
            double link_width, 
            const BoundedValue& theta_1, const BoundedValue& theta_2, const BoundedValue& theta_3);

    // Copy Constructor
    Robot(const Robot& robot)
    : link_0_length_{robot.link_0_length_}, link_1_length_{robot.link_1_length_}, link_2_length_{robot.link_2_length_}, link_3_length_{robot.link_3_length_},
        link_width_{robot.link_width_},
        theta_1_{robot.theta_1_}, theta_2_{robot.theta_2_}, theta_3_{robot.theta_3_}, 
        link_0_{robot.link_0_}, link_1_{robot.link_1_}, link_2_{robot.link_2_}, link_3_{robot.link_3_}, link_ee_{robot.link_ee_},
        collision_objects_{robot.collision_objects_}
    {
    }

    // Move constructor
    Robot(const Robot&& robot)
    : link_0_length_{robot.link_0_length_}, link_1_length_{robot.link_1_length_}, link_2_length_{robot.link_2_length_}, link_3_length_{robot.link_3_length_},
        link_width_{robot.link_width_},
        theta_1_{robot.theta_1_}, theta_2_{robot.theta_2_}, theta_3_{robot.theta_3_}, 
        link_0_{robot.link_0_}, link_1_{robot.link_1_}, link_2_{robot.link_2_}, link_3_{robot.link_3_}, link_ee_{robot.link_ee_},
        collision_objects_{robot.collision_objects_}
    {
    }

    // Move assignment operator
    Robot& operator=(Robot&& other);

    // Copy assignment operator
    Robot& operator=(const Robot& other);

    void setJointValues(const double theta1, const double theta2, const double theta3);

    void setJointValues(const std::array<double,3> thetas);

    std::array<BoundedValue, 3> getJointValues(){
        return {theta_1_, theta_2_, theta_3_};
    }

    const Pose2D getEEPose() const;

    const Pose2D getLinkPose(const Link& link) const;

    const Pose2D getEEPose(const double theta1, const double theta2, const double theta3);

    void getRobotCollisionObjects(std::vector<Box>& collision_objects) const;

    // Collision objects of the Robot when it has grasp the subject
    void getRobotCollisionObjects(Box& subject, std::vector<Box>& collision_objects) const;


    // Calculation of inverseKinematics
    bool inverseKinematics(Pose2D ee_pose, std::array<double, 3>& joint_values);


    void Draw(sf::RenderWindow& window, const std::pair<double, double> scale) const;
};


#endif
