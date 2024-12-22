#ifndef __UTILITY_HPP__
#define __UTILITY_HPP__

#include <algorithm>
#include <vector>
#include <array>
#include <cmath>
#include <limits>
#include <iostream>
#include <random>
#include <utility>

#include <SFML/Graphics.hpp>

/* 2D Point
* A structure to represent a 2D Point
*/
struct Point2D{
    double x{0.0};
    double y{0.0};
};
using Vector2D = Point2D;

/* Pose
* A structure to present the Pose
*/
struct Pose2D{
    Point2D position;
    double orientation{0.0};
};

std::ostream& operator<<(std::ostream& os, const Pose2D& pose);


/* BoubdedValue
* A class to present a bounded value
*/
class BoundedValue {

private:
    double value_;
    double min_value_;
    double max_value_;

    // Helper function to clamp the value within bounds
    double clamp(double val) const {
        return std::max(min_value_, std::min(max_value_, val));
    }

public:

    BoundedValue()
        : min_value_(0.0), max_value_(0.0) , value_(0.0)
    {}
    // Constructor to initialize min, max, and initial value
    BoundedValue(double min_value, double max_value, double initial_value)
        : min_value_(min_value), max_value_(max_value) {
        this->value_ = this->clamp(initial_value);
    }

    // Copy constructor
    BoundedValue(const BoundedValue& other)
        : value_(other.value_), min_value_(other.min_value_), max_value_(other.max_value_) {}

    // Assignment operator from another BoundedValue
    BoundedValue& operator=(const BoundedValue& other) {
        if (this != &other) {
            value_ = other.value_;
            min_value_ = other.min_value_;
            max_value_ = other.max_value_;
        }
        return *this;
    }


    bool isWithinBounds(double val) const{
        return( (val>=this->min_value_) && (val<=this->max_value_));
    }

    // Assignment operator from T
    BoundedValue& operator=(const double new_value) {
        value_ = clamp(new_value);
        return *this;
    }

    // Arithmetic operators with another BoundedValue
    BoundedValue operator+(const BoundedValue& other) const {
        return BoundedValue(min_value_, max_value_, value_ + other.value_);
    }
    BoundedValue operator-(const BoundedValue& other) const {
        return BoundedValue(min_value_, max_value_, value_ - other.value_);
    }
    BoundedValue operator*(const BoundedValue& other) const {
        return BoundedValue(min_value_, max_value_, value_ * other.value_);
    }
    BoundedValue operator/(const BoundedValue& other) const {
        return BoundedValue(min_value_, max_value_, value_ / other.value_);
    }

    BoundedValue operator+(double other) const {
        return BoundedValue(min_value_, max_value_, value_ + other);
    }
    BoundedValue operator-(double other) const {
        return BoundedValue(min_value_, max_value_, value_ - other);
    }
    BoundedValue operator*(double other) const {
        return BoundedValue(min_value_, max_value_, value_ * other);
    }
    BoundedValue operator/(double other) const {
        return BoundedValue(min_value_, max_value_, value_ / other);
    }

    // Compound assignment operators with another BoundedValue
    BoundedValue& operator+=(const BoundedValue& other) {
        value_ = clamp(value_ + other.value_);
        return *this;
    }
    BoundedValue& operator-=(const BoundedValue& other) {
        value_ = clamp(value_ - other.value_);
        return *this;
    }
    BoundedValue& operator*=(const BoundedValue& other) {
        value_ = clamp(value_ * other.value_);
        return *this;
    }
    BoundedValue& operator/=(const BoundedValue& other) {
        value_ = clamp(value_ / other.value_);
        return *this;
    }

    // Compound assignment operators with T
    BoundedValue& operator+=(double other) {
        value_ = clamp(value_ + other);
        return *this;
    }
    BoundedValue& operator-=(double other) {
        value_ = clamp(value_ - other);
        return *this;
    }
    BoundedValue& operator*=(double other) {
        value_ = clamp(value_ * other);
        return *this;
    }
    BoundedValue& operator/=(double other) {
        value_ = clamp(value_ / other);
        return *this;
    }

    // Comparison operators with another BoundedValue
    bool operator==(const BoundedValue& other) const { return value_ == other.value_; }
    bool operator!=(const BoundedValue& other) const { return value_ != other.value_; }
    bool operator<(const BoundedValue& other) const { return value_ < other.value_; }
    bool operator>(const BoundedValue& other) const { return value_ > other.value_; }
    bool operator<=(const BoundedValue& other) const { return value_ <= other.value_; }
    bool operator>=(const BoundedValue& other) const { return value_ >= other.value_; }

    // Comparison operators with T
    bool operator==(double other) const { return value_ == other; }
    bool operator!=(double other) const { return value_ != other; }
    bool operator<(double other) const { return value_ < other; }
    bool operator>(double other) const { return value_ > other; }
    bool operator<=(double other) const { return value_ <= other; }
    bool operator>=(double other) const { return value_ >= other; }

    // Unary operators
    BoundedValue operator+() const {
        return *this;
    }
    BoundedValue operator-() const {
        return BoundedValue(min_value_, max_value_, clamp(-value_));
    }

    // Increment and decrement operators
    BoundedValue& operator++() { // Prefix increment
        value_ = clamp(value_ + 1);
        return *this;
    }
    BoundedValue operator++(int) { // Postfix increment
        BoundedValue temp(*this);
        value_ = clamp(value_ + 1);
        return temp;
    }
    BoundedValue& operator--() { // Prefix decrement
        value_ = clamp(value_ - 1);
        return *this;
    }
    BoundedValue operator--(int) { // Postfix decrement
        BoundedValue temp(*this);
        value_ = clamp(value_ - 1);
        return temp;
    }

    // Getter and setter methods
    void setValue(double newValue) {
        value_ = clamp(newValue);
    }
    double getValue() const {
        return value_;
    }

    double getRandomValue() const{
        std::random_device rd;  // Seed generator
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dis(this->min_value_, this->max_value_);
        return dis(gen);
    }


};


/* Box
* A class to represent a Box in 2D
* The Box is represented by the Pose of Mid-Left point
* and its width and height
*/
class Box {
private:
    // Mid-Left point
    Pose2D pose_;
    // Width of the box   
    double width_;
    // Height of the box
    double height_;  

public:

    // Constructor
    Box(const Pose2D& pose /*Mid-Left point*/, double width, double height)
        : pose_{pose}, width_{width}, height_{height} {}

    const Pose2D getPose() const { return pose_; }

    double getWidth() const { return width_; }

    double getHeight() const { return height_; }

    // Get the four corners of the box
    std::vector<Point2D> getCorners() const {
        // Half dimensions
        double half_height = height_ / 2.0;

        // Define the corners relative to the midpoint of the left side
        // (before rotation)
        std::vector<Point2D> corners = {
            {0.0, half_height},              // Upper-left
            {width_, half_height},           // Upper-right
            {width_, -half_height},          // Lower-right
            {0.0, -half_height}              // Lower-left
        };

        // Rotate and translate the corners
        std::vector<Point2D> transformed_corners(4);
        double cos_theta = cos(pose_.orientation);
        double sin_theta = sin(pose_.orientation);

        std::transform(corners.begin(), corners.end(),transformed_corners.begin(),
                        [cos_theta, sin_theta, this](const Point2D& point)->Point2D{
                            Point2D result;
                            result.x = point.x * cos_theta - point.y * sin_theta + pose_.position.x;
                            result.y = point.x * sin_theta + point.y * cos_theta + pose_.position.y;
                            return result;
                        });

        return transformed_corners;
    }

    void setPose(const Pose2D& pose) { pose_ = pose;}

};

// Euclidean Distance
double distance(const std::array<double,3>& q1, const std::array<double,3>& q2) ;


/* minDistanceBetweenLineSegments
* A function to present the minimum distance 
* between two line segments represented by beginning and end point 
*/
double minDistanceBetweenLineSegments(const Point2D& p1, const Point2D& p2,
                                      const Point2D& q1, const Point2D& q2);

/* minDistanceBetweenBoxes
* Finding minimum distance between two boxes.
*/
double minDistanceBetweenBoxes(const Box& box1, const Box& box2);

/*areBoxesClear
* Determining if two Boxes are clear of each others
*/
bool areBoxesClear(Box box1, Box box2, double clearance = 0.0);


#endif
