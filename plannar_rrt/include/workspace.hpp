#ifndef __WORKSPACE_HPP__
#define __WORKSPACE_HPP__

#include <vector>
#include <memory>
#include "robot.hpp"
#include "utility.hpp"
#include <sstream>
#include <iomanip>
#include <iostream>

#include <SFML/Graphics.hpp>


class Workspace {
private:

    // Width of the workspace
    double width_;   
    // Height of the workspace 
    double height_; 

    Robot& robot_;
  
    std::vector<Box> obstacles_;

    std::shared_ptr<Box> subject_{nullptr};

    Pose2D target_pose_;

public:
    // Constructor
    Workspace(double width, double height, Robot& robot)
        : width_{width}, height_{height}, robot_{robot}
    {}

    // Method to add a box to the workspace
    void addObstacle(const Box& box) {
        obstacles_.push_back(box);
    }

    // Method to get all obstacles
    const std::vector<Box>& getObstacles() const {
        return obstacles_;
    }

    // Get workspace dimensions
    double getWidth() const { return width_; }
    double getHeight() const { return height_; }


    Robot& getRobot(){
        return robot_;
    }

    void setSubject(const Box& box){
        subject_ = std::make_unique<Box>(box);
    }

    Box getSubject(){
        return *subject_;
    }

    void setTargetPose(const Pose2D& target_pose){
        this->target_pose_ = target_pose;
    }

    const Pose2D getTargetPose() const{
        return this->target_pose_;
    }

    bool isThereCollision(bool grasped = false, double clearance = 0.0);

    void Draw();
};


void saveFrame(sf::RenderWindow& window, int frameNumber);

void animateTrajectory(sf::RenderWindow& window, Workspace& workspace, Robot& robot, 
                       const std::vector<std::array<double, 3>>& trajectory, 
                       Box& subject, const std::pair<double, double>& scale, 
                       const sf::Color& subjectColor, int& frameCounter);



#endif