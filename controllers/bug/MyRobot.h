#ifndef MY_ROBOT_H_
#define MY_ROBOT_H_

/**
 * @file    MyRobot.h
 * @brief   A simple example for maintaining a straight line with the compass.
 *
 * @author  Sara Marqués Villarroya <smarques@ing.uc3m.es>
 * @author  Juan José Gamboa Montero <jgamboa@ing.uc3m.es>
 * @date    2020-10
 */

#include <iostream>
#include <limits>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Compass.hpp>
#include <webots/DistanceSensor.hpp>
#include <math.h>

using namespace std;
using namespace webots;

#define MAX_SPEED       10
#define DESIRED_ANGLE   179


//////////////////////////////////////////////

#define NUM_DISTANCE_SENSOR 6
#define DISTANCE_LIMIT 200
#define SHORT_LIMIT 900
#define MAX_SPEED 10

// working modes
enum RobotMode {
  STOP,
  FORWARD,
  TURN_LEFT,
  TURN_RIGHT,
  OBSTACLE_AVOID,
  ORIENT,
  WALL_FOLLOW,
};

//////////////////////////////////////////////
        
class MyRobot : public Robot {
    public:
        /**
         * @brief Empty constructor of the class.
         */
        MyRobot();

        /**
         * @brief Destructor of the class.
         */
        ~MyRobot();

        /**
         * @brief Function with the logic of the controller.
         * @param
         * @return
         */
        void run();
        
        /**
         * @brief Lock state of robot for a certain number of ticks
         */
        void lock_state(int ticks);

        /**
         * @brief Function for converting bearing vector from compass to angle (in degrees).
         * @param in_vector Input vector with the values to convert.
         * @return The value in degrees.
         */
        double convert_bearing_to_degrees(const double* in_vector);
  
        /**
         * Print robot current state to cout.
         */
        void print_mode();

    private:
        // Whether to print verbose
        bool verbose;
        
        // The time step
        int _time_step;
        
        // how many ticks the robot will be locked in state for
        int _state_timer;
        
        // which direction to back the robot back in.
        bool _backing_direction;
        
        // velocities
        double _left_speed, _right_speed;

        // Sensors
        Compass *_my_compass;
        
        // Motors
        Motor *_left_wheel_motor;
        Motor *_right_wheel_motor;
        
        RobotMode _mode;
        
        DistanceSensor*_distance_sensor[NUM_DISTANCE_SENSOR];
        const char *ds_name[NUM_DISTANCE_SENSOR] = {"ds1", 
        "ds14", "ds11", "ds12", "ds3", "ds4"};
        const char* mode_names[7] = {"STOP", "FORWARD", "BACK_LEFT", "BACK_RIGHT", "OBSTACLE_AVOID", "ORIENT", "WALL FOLLOW"};
};

#endif

