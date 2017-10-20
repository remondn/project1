#include <project1/pid.h>
#include <math.h>
#include <iostream>

PID::PID(){

    /* TO DO
     *
     * find appropriate value of the coefficients for PID control, and initialize them.
     *
    */

    // PID Parameters : Values from Wikipedia
    Kp = 5.0;
    Ki = 3.0;
    Kd = 3.0;

    // Intern variable
    error = 0.0;
    error_sum = 0.0;
    lastError;

}

float PID::get_control(point car_pose, point goal_pose){

    float ctrl;
    double max_turn = 60.0*M_PI/180.0;
    float pt;
    float it;
    float dt;

    /* TO DO
     *
     * implement pid algorithm
     *
    */

    // Return the closest angle with 0
    if(car_pose.th < -M_PI)
    {
        car_pose.th += 2 * M_PI;  // Equivalent
    }
    if(car_pose.th > M_PI)
    {
        car_pose.th -= 2 * M_PI;  // Equivalent
    }

    // Compute e(t) from the point of the car and the goal point
    error = getDirection(car_pose, goal_pose) - car_pose.th;

    //return error;

    // Compute the proportionnal term
    pt = Kp * error;

    // Compute the Integral term
    it = Ki * DELTA_T * error_sum;

    // Compute Derivative term
    dt = (Kd / DELTA_T) * (error - lastError);

    // Recursion form
    ctrl = pt + it + dt;

    // We have the output of PID algorithm, but we need to set variables for next loop
    lastError = error;
    error_sum += error;

    // Take care of min and max
    if(ctrl > max_turn)
    {
        ctrl = max_turn;
    }
    if(ctrl < -max_turn)
    {
        ctrl = -max_turn;
    }

    //DEBUG
    std::cout << "------------- DATA PID ----------------" << std::endl;
    std::cout << "Car Position = (" << car_pose.x << ";" << car_pose.y << ")" << std::endl;
    std::cout << "Goal Position = (" << goal_pose.x << ";" << goal_pose.y << ")" << std::endl;
    std::cout << "Goal Position / car = (" << goal_pose.x + car_pose.x << ";" << goal_pose.y + car_pose.y << ")" << std::endl;    
    std::cout << "Goal orientation = " << getDirection(car_pose, goal_pose) << std::endl;
    std::cout << "Car orientation = " << car_pose.th << std::endl;
    std::cout << "Error = " << error << std::endl;
    std::cout << "Ctrl = " << ctrl << std::endl;
    std::cout << "---------------------------------------" << std::endl;

    return ctrl;
}

float PID::getDirection(point car_pose, point goal_pose)
{
    float opposite;
    float adjacent;

    // To not have problems with infinity
    if(car_pose.x == goal_pose.x && car_pose.y == goal_pose.y)
    {
        return 0.0;
    }

    opposite = goal_pose.x - car_pose.x;
    adjacent = goal_pose.y - car_pose.y;

    // Compute the angle
    float angle = atan(opposite / adjacent);

    // Having the same direction doesn't mean we have the same angle : ajust it
    if(adjacent < 0.0)
    {
        angle += M_PI - angle;
    }
    //if(opposite < 0.0)
    //{
    //    angle = M_PI - angle;
    //}

    // Return the closest angle with 0
    if(angle < -M_PI)
    {
        angle += 2 * M_PI;  // Equivalent
    }
    if(angle > M_PI)
    {
        angle -= 2 * M_PI;  // Equivalent
    }
    

    return angle;
}

