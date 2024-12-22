#include "workspace.hpp"


bool Workspace::isThereCollision(bool grasped, double clearance){
    // Check is the any Links of the Robot and the possible subject in hand
    // with any of the Boxes in the scene
    std::vector<Box> robot_collision_objects;
    if(grasped){
        // If the subject is grasped it is part of the robot collisiton object
        this->robot_.getRobotCollisionObjects(*subject_, robot_collision_objects);
        for(auto& box1: robot_collision_objects){
            for(auto& box2: obstacles_){
                if(!areBoxesClear(box1, box2, clearance)){
                    return true;
                }
            }
        }
    }else{
        // If the subject is grasped it is part of the environment collisiton object
        this->robot_.getRobotCollisionObjects(robot_collision_objects);
        for(auto& box1: robot_collision_objects){
            for(auto& box2: obstacles_){
                if(!areBoxesClear(box1, box2, clearance)){
                    return true;
                }
            }
            if(!areBoxesClear(box1, *subject_, clearance)){
                return true;
            }
        }
    }
    return false;
}


void Workspace::Draw() {
    constexpr double WINDOW_WIDTH{800.0};
    constexpr double WINDOW_HEIGHT{800.0};
    constexpr int FRAME_RATE{60};

    const double scale_x = WINDOW_WIDTH / width_;
    const double scale_y = WINDOW_HEIGHT / height_;

    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Workspace");
    window.setFramerateLimit(FRAME_RATE);

    while (window.isOpen()) {
        window.clear(sf::Color::White);

        // Draw obstacles
        for (const Box& obstacle : obstacles_) {
            sf::RectangleShape box(sf::Vector2f(
                static_cast<float>(obstacle.getWidth() * scale_x),
                static_cast<float>(obstacle.getHeight() * scale_y)
            ));
            box.setOrigin(0, static_cast<float>(obstacle.getHeight() * scale_y / 2.0));
            box.setPosition(
                static_cast<float>(obstacle.getPose().position.x * scale_x),
                static_cast<float>((1.0 - obstacle.getPose().position.y / height_) * WINDOW_HEIGHT)
            );
            box.setRotation(static_cast<float>(obstacle.getPose().orientation * 180.0 / M_PI));
            box.setFillColor(sf::Color::Red);
            window.draw(box);
        }

        // Handle events
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }

        // Display everything
        window.display();
    }
}

void saveFrame(sf::RenderWindow& window, int frameNumber) {
    sf::Texture texture;
    texture.create(window.getSize().x, window.getSize().y);
    texture.update(window);

    // Generate file name with zero-padded frame number
    std::ostringstream filename;
    filename << "frame_" << std::setfill('0') << std::setw(5) << frameNumber << ".png";

    // Save the current frame as an image
    if (!texture.copyToImage().saveToFile(filename.str())) {
        std::cerr << "Failed to save frame " << frameNumber << "\n";
    }
}

void animateTrajectory(sf::RenderWindow& window, Workspace& workspace, Robot& robot, 
                       const std::vector<std::array<double, 3>>& trajectory, 
                       Box& subject, const std::pair<double, double>& scale, 
                       const sf::Color& subjectColor, int& frameCounter) {
    constexpr int FRAME_DELAY = 10; // Milliseconds delay between frames

    for (const auto& config : trajectory) {
        // Update the robot's joint values
        robot.setJointValues(config);

        // Move the subject along with the robot's end effector
        subject.setPose(robot.getEEPose());

        // Clear the window
        window.clear(sf::Color::White);

        // Draw obstacles
        for (const auto& obstacle : workspace.getObstacles()) {
            sf::RectangleShape rect(sf::Vector2f(
                static_cast<float>(obstacle.getWidth() * scale.first),
                static_cast<float>(obstacle.getHeight() * scale.second)
            ));
            rect.setOrigin(0, static_cast<float>(obstacle.getHeight() * scale.second / 2.0));
            rect.setPosition(
                static_cast<float>((obstacle.getPose().position.x + 5.0) * scale.first),
                static_cast<float>((-obstacle.getPose().position.y + 5.0) * scale.second)
            );
            rect.setRotation(static_cast<float>(obstacle.getPose().orientation * 180.0 / M_PI));
            rect.setFillColor(sf::Color::Red);
            window.draw(rect);
        }

        // Draw the subject
        sf::RectangleShape subject_rect(sf::Vector2f(
            static_cast<float>(subject.getWidth() * scale.first),
            static_cast<float>(subject.getHeight() * scale.second)
        ));
        subject_rect.setOrigin(0, static_cast<float>(subject.getHeight() * scale.second / 2.0));
        subject_rect.setPosition(
            static_cast<float>((subject.getPose().position.x + 5.0) * scale.first),
            static_cast<float>((-subject.getPose().position.y + 5.0) * scale.second)
        );
        subject_rect.setRotation(static_cast<float>(subject.getPose().orientation * 180.0 / M_PI));
        subject_rect.setFillColor(subjectColor);
        window.draw(subject_rect);

        // Draw the robot
        workspace.getRobot().Draw(window, scale);

        // Display the updated frame
        window.display();

        // Save the current frame
        saveFrame(window, frameCounter++);

        // Pause to create animation effect
        sf::sleep(sf::milliseconds(FRAME_DELAY));
    }
}
