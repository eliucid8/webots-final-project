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
#include <webots/PositionSensor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/GPS.hpp>
#include <webots/Camera.hpp>
#include <math.h>

using namespace std;
using namespace webots;

#define MAX_SPEED       10

#define WHEELS_DISTANCE 0.3606   //[=] meters
#define WHEEL_RADIUS    0.0825 //[=] meters

#define ENCODER_TICS_PER_RADIAN 1

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
  YELLOW_CROSS
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
         * @brief Go to point
         */
        void go_to_point(double x, double y);
        
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
        
        /**
         * @brief Updates the odometry of the robot in meters and radians. The atributes _x, _y, _theta are updated.
         */
        void compute_odometry();

        /**
         * @brief Prints in the standard output the x,y,theta coordinates of the robot.
         */
        void print_odometry();
        
        /**
         * @brief Prints in the standard output the x,y,theta coordinates of the robot.
         * This method uses the encoder resolution and the wheel radius defined in the model of the robot.
         *
         * @param tics raw value read from an encoder
         * @return meters corresponding to the tics value
         */
        float encoder_tics_to_meters(float tics);
            
        /**
         * @brief Recalculates _theta_goal with current position information
         */
        void update_theta_goal();
        
        /**
         * @brief Checks whether the robot has reached the goal established for this controller.
         * @return true if the robot has reached the goal established for this controller; false otherwise.
         */
        bool goal_reached();

        bool detect_yellow_line();

        bool detect_victim();

        // allow robot to track and follow victim by camera
        double center_victim();

        bool _passed_yellow_line;

        bool has_crossed_yellow_line();

        bool returning_rescue;

        bool in_cooldown;

        int cooldown_counter;

        int victim_count;

        void handle_victim_detection();

        void spin_in_place();

        static const int COOLDOWN_STEPS = 1000 / 64;  // 1 seconds

        bool bug_to_victim();

        void turn_relative_angle(double degrees);

        bool performed_turnaround;

        void check_and_handle_turnaround();

        void rescue_procedure();

        void wall_follow_step();

        bool angular_range_check(double low, double high, double check);


    private:
        // Whether to print verbose
        bool verbose;
        
        // The time step
        int _time_step;
        
        // which direction to back the robot back in.
        bool _backing_direction;
        
        // velocities
        double _left_speed, _right_speed;
        
        double _x, _y, _x_goal, _y_goal;   // [=] meters
        double _theta, _theta_goal;   // [=] rad
        
        float _sr, _sl;  // [=] meters

        // Sensors
        Compass *_my_compass;
        
        // Motors
        Motor *_left_wheel_motor;
        Motor *_right_wheel_motor;

        // Motor Position Sensor
        PositionSensor* _left_wheel_sensor;
        PositionSensor* _right_wheel_sensor;

        // GPS sensor
        GPS* _my_gps;

        DistanceSensor *_distance_sensor[NUM_DISTANCE_SENSOR];
        const char *ds_name[NUM_DISTANCE_SENSOR] = {"ds1", 
        "ds14", "ds11", "ds12", "ds3", "ds4"};
        
        Camera* _front_cam; 
        Camera* _spher_cam;

        const char* mode_names[8] = {"STOP", "FORWARD", "BACK_LEFT", "BACK_RIGHT", "OBSTACLE_AVOID", "ORIENT", "WALL FOLLOW", "YELLOW_CROSS"};

        RobotMode _mode;
};

#endif

