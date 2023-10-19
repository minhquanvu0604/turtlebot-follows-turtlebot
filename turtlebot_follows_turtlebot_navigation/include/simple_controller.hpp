#ifndef SIMPLE_CONTROLLER_HPP
#define SIMPLE_CONTROLLER_HPP

#include "controller.hpp"

// Concrete SimpleController class
class SimpleController : public Controller {
public:
    SimpleController(double desired_distance, double max_linear_speed, double max_angular_speed);
    void compute_velocity_commands(double current_distance, double angle_to_tag, double &linear_velocity, double &angular_velocity) override;

private:
    double desired_distance_;
    double max_linear_speed_;
    double max_angular_speed_;
};

#endif // SIMPLE_CONTROLLER_HPP
