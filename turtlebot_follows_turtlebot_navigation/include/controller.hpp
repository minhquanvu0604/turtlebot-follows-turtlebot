#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

// Abstract Controller class
class Controller {
public:
    virtual ~Controller() {}
    virtual void compute_velocity_commands(double current_distance, double angle_to_tag, double &linear_velocity, double &angular_velocity) = 0;
};

#endif // CONTROLLER_HPP
