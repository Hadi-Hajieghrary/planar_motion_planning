#include "utility.hpp"

std::ostream& operator<<(std::ostream& os, const Pose2D& pose){
    os<<"X: "<<pose.position.x<<" "<<", Y: "<<pose.position.y<<", Theta: "<<pose.orientation;
    return os;
} 



double distance(const std::array<double,3>& q1, const std::array<double,3>& q2) {
    return std::sqrt(std::pow(q1[0] - q2[0], 2) +
                     std::pow(q1[1] - q2[1], 2) +
                     std::pow(q1[2] - q2[2], 2));
}


double minDistanceBetweenLineSegments(const Point2D& p1, const Point2D& p2,
                                      const Point2D& q1, const Point2D& q2)
{
    auto subtract = [](const Point2D& p1, const Point2D& p2) -> Vector2D {
        return Vector2D{p1.x - p2.x, p1.y - p2.y};
    };

    auto dot = [](const Point2D& p1, const Point2D& p2) -> double {
        return p1.x * p2.x + p1.y * p2.y;
    };

    auto lengthSquared = [](const Vector2D& v) {
        return v.x * v.x + v.y * v.y;
    };

    Vector2D d1 = subtract(p2, p1); // Vector from p1 to p2
    Vector2D d2 = subtract(q2, q1); // Vector from q1 to q2
    Vector2D r = subtract(p1, q1);  // Vector from q1 to p1

    double a = lengthSquared(d1); // Squared length of segment p1-p2
    double e = lengthSquared(d2); // Squared length of segment q1-q2
    double f = dot(d2, r);        // Dot product of d2 and r

    const double EPSILON = 1e-9;
    double s = 0.0, t = 0.0;

    // Check if both segments are points
    if (a <= EPSILON && e <= EPSILON) {
        return std::sqrt(lengthSquared(r));
    }

    if (a <= EPSILON) {
        // Segment p1-p2 is a point
        t = std::max(0.0, std::min(1.0, f/e));
    } else {
        double c = dot(d1, r);
        if (e <= EPSILON) {
            // Segment q1-q2 is a point
            s = std::max(0.0, std::min(1.0, -c/a));
        } else {
            // General case: compute s and t
            double b = dot(d1, d2);
            double denom = a * e - b * b;
            if (std::abs(denom) > EPSILON) {
                s = std::max(0.0, std::min(1.0, (b * f - c * e) / denom));
            } else {
                s = 0.0; // Parallel lines
            }
            t = std::max(0.0, std::min(1.0, (b * s + f) / e));
        }
    }

    // Compute closest points
    Point2D closestPointP = {p1.x + s * d1.x, p1.y + s * d1.y};
    Point2D closestPointQ = {q1.x + t * d2.x, q1.y + t * d2.y};

    return std::sqrt(lengthSquared(subtract(closestPointP, closestPointQ)));
}

double minDistanceBetweenBoxes(const Box& box1, const Box& box2) {
    // Get the corners of both boxes
    auto corners1 = box1.getCorners();
    auto corners2 = box2.getCorners();

    double minDist = std::numeric_limits<double>::infinity();

    // Edges of box1
    for (int i = 0; i < 4; ++i) {
        const auto& p1_start = corners1[i];
        const auto& p1_end = corners1[(i + 1) % 4];

        // Edges of box2
        for (int j = 0; j < 4; ++j) {
            const auto& p2_start = corners2[j];
            const auto& p2_end = corners2[(j + 1) % 4];

            double dist = minDistanceBetweenLineSegments(p1_start, p1_end, p2_start, p2_end);
            if (dist < minDist) {
                minDist = dist;
            }

            // Early exit if distance is zero (boxes are touching or overlapping)
            if (minDist == 0.0) {
                return 0.0;
            }
        }
    }

    return minDist;
}


bool areBoxesClear(Box box1, Box box2, double clearance){
    double minDist = minDistanceBetweenBoxes(box1, box2);
    if (minDist > clearance) {
        return true;
    }else{
        return false;
    }
}


void visualizeTrajectory(sf::RenderWindow& window, const std::vector<std::array<double, 3>>& trajectory, const double scale, const sf::Color& color) {
    for (size_t i = 1; i < trajectory.size(); ++i) {
        // Draw a dotted line for each joint
        for (int joint = 0; joint < 3; ++joint) {
            sf::CircleShape dot(2);  // Small circle for each point in the trajectory
            dot.setFillColor(color);

            // Scale and position the trajectory point
            double x = trajectory[i][joint] * scale + (joint * 50);  // Offset for each joint
            double y = trajectory[i - 1][joint] * scale + 300;       // Vertical center offset
            dot.setPosition(static_cast<float>(x), static_cast<float>(y));
            window.draw(dot);
        }
    }
}

