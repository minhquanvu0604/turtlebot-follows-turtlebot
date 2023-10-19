#include "simple_controller.hpp"
#include <cmath>

SimpleController::SimpleController(double desired_distance, double max_linear_speed, double max_angular_speed)
    : desired_distance_(desired_distance), 
      max_linear_speed_(max_linear_speed), 
      max_angular_speed_(max_angular_speed) {}

void SimpleController::compute_velocity_commands(double current_distance, double angle_to_tag, double &linear_velocity, double &angular_velocity) {
    // Simple proportional control to maintain the desired distance
    linear_velocity = (current_distance - desired_distance_) * 0.5;
    if (linear_velocity > max_linear_speed_) {
        linear_velocity = max_linear_speed_;
    } else if (linear_velocity < -max_linear_speed_) {
        linear_velocity = -max_linear_speed_;
    }

    // Simple proportional control for angle correction
    angular_velocity = -angle_to_tag * 0.1;
    if (angular_velocity > max_angular_speed_) {
        angular_velocity = max_angular_speed_;
    } else if (angular_velocity < -max_angular_speed_) {
        angular_velocity = -max_angular_speed_;
    }
}
