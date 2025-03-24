/**
 * @file    MyRobot.cpp
 * @brief   A simple example for maintaining a straight line with the compass.
 *
 * @author  Sara Marqués Villarroya <smarques@ing.uc3m.es>
 * @author  Juan José Gamboa Montero <jgamboa@ing.uc3m.es>
 * @date    2020-10
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : Robot()
{
    // init default values
    verbose = true;
    _time_step = 64;
    // initialize motor velocities
    _left_speed = 0;
    _right_speed = 0;
    // set mode to forward motion
    _mode = FORWARD;

    _backing_direction = true;
    // get distance sensor array and enable each one
    for (int i = 0; i < NUM_DISTANCE_SENSOR; i++)
    {
        _distance_sensor[i] = getDistanceSensor(ds_name[i]);
        _distance_sensor[i]->enable(_time_step);
    }
    // _distance_sensor[0] = getDistanceSensor("ds1");
    // _distance_sensor[0]->enable(_time_step);
    // _distance_sensor[1] = getDistanceSensor("ds14");
    // _distance_sensor[1]->enable(_time_step);
    // _distance_sensor[2] = getDistanceSensor("ds11");
    // _distance_sensor[2]->enable(_time_step);
    // _distance_sensor[3] = getDistanceSensor("ds12");
    // _distance_sensor[3]->enable(_time_step);
    // motor initialization
    _left_wheel_motor = getMotor("left wheel motor");
    _right_wheel_motor = getMotor("right wheel motor");
    // set the motor position to non-stop moving -> Velocity Control
    _left_wheel_motor->setPosition(INFINITY);
    _right_wheel_motor->setPosition(INFINITY);

    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);

    _state_timer = 0;
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    // disable devices
    _my_compass->disable();
}

//////////////////////////////////////////////

void MyRobot::run()
{
    double ir1_val = 0.0, ir14_val = 0.0;
    double back_right = 0.0, front_right = 0.0; // initialize distance sensor values
    double back_left = 0.0, front_left = 0.0;
    double compass_angle = 0.0;
    while (step(_time_step) != -1)
    {
        // read the sensors
        ir1_val = _distance_sensor[0]->getValue();
        ir14_val = _distance_sensor[1]->getValue();
        back_right = _distance_sensor[2]->getValue();
        front_right = _distance_sensor[3]->getValue();
        back_left = _distance_sensor[4]->getValue();
        front_left = _distance_sensor[5]->getValue();
        double front_avg = (ir1_val + ir14_val) * 0.5;

        const double *compass_val = _my_compass->getValues();
        compass_angle = convert_bearing_to_degrees(compass_val);

        if (verbose)
        {
            // cout << ir1_val << ", " << ir14_val << endl;
            // cout << "Compass angle (degrees): " << compass_angle << endl;
        }
        RobotMode new_mode = _mode;

        if ((_mode == OBSTACLE_AVOID && front_avg > 0) || _state_timer > 0)
        {
            if(verbose) {
                cout << "State locked: " << _state_timer << endl;
            }
            --_state_timer;
        }
        else
        {
            // default case
            new_mode = FORWARD;
            // control logic of the robot
            if (back_right > DISTANCE_LIMIT || front_right > DISTANCE_LIMIT || back_left > DISTANCE_LIMIT || front_left > DISTANCE_LIMIT)
            {
                new_mode = WALL_FOLLOW;
            }

            if (front_right == 0 && back_right == 0 && ir1_val == 0 && ir14_val == 0)
            {
                if (compass_angle < 175.0 || compass_angle > 185.0)
                {
                    new_mode = ORIENT;
                    
                }
            }

            // TODO: only back like this if we have lost track of the wall
            if (_mode != OBSTACLE_AVOID && ((ir1_val > DISTANCE_LIMIT) || (ir14_val > DISTANCE_LIMIT)))
            {
                cout << _mode << endl;
                new_mode = OBSTACLE_AVOID;
                _backing_direction = !_backing_direction;
                lock_state(30);
            }
            
            // FOR TURNING INSIDE CORNER
            if ((back_right > DISTANCE_LIMIT && front_right > DISTANCE_LIMIT) && (front_avg > DISTANCE_LIMIT)) {
                new_mode = TURN_LEFT;
                lock_state(50);
            }

            if ((back_left > DISTANCE_LIMIT && front_right > DISTANCE_LIMIT) && (front_avg > DISTANCE_LIMIT)) {
                new_mode = TURN_RIGHT;
                lock_state(50);
            }
            
            _mode = new_mode;
        }
        
        // turn speeds
        double forward_speed = MAX_SPEED / 4.0;
        double backward_speed = -MAX_SPEED / 5.0;
        double mult_factor = min(1.0, abs(compass_angle - 180.0) / 45.0);

        // send actuators commands according to the mode
        switch (_mode)
        {
        case FORWARD:
            cout << "Moving forward." << endl;
            _left_speed = MAX_SPEED * 0.5;
            _right_speed = MAX_SPEED * 0.5;
            break;
        case OBSTACLE_AVOID:
            if (_backing_direction)
            {
                cout << "Backing up and turning left." << endl;
                _left_speed = -MAX_SPEED / 6.0;
                _right_speed = MAX_SPEED / 9.0;
            }
            else
            {
                cout << "Backing up and turning right." << endl;
                _right_speed = -MAX_SPEED / 6.0;
                _left_speed = MAX_SPEED / 9.0;
            }
            break;
        case WALL_FOLLOW:
            if((back_right > DISTANCE_LIMIT && back_right > front_right) || (front_left > DISTANCE_LIMIT && back_left < front_left)) {
                _left_speed = MAX_SPEED * 0.3;
                _right_speed = MAX_SPEED * 0.15;
            }
            if((front_right > DISTANCE_LIMIT && back_right < front_right) || (back_left > DISTANCE_LIMIT && back_left > front_left)) {
                _left_speed = MAX_SPEED * 0.15;
                _right_speed = MAX_SPEED * 0.3;
            }
            break;
        case TURN_LEFT:
            cout << "turning left." << endl;
            _left_speed = -MAX_SPEED / 6.0;
            _right_speed = MAX_SPEED / 9.0;
            break;
        case TURN_RIGHT:
            cout <<  "turning right." << endl;
            _right_speed = -MAX_SPEED / 6.0;
            _left_speed = MAX_SPEED / 9.0;
            break;
        case ORIENT:
            forward_speed *= mult_factor;
            backward_speed *= mult_factor;

            if (compass_angle <= 178.0)
            {
                _left_speed = backward_speed;
                _right_speed = forward_speed;
            }
            if (compass_angle >= 182.0)
            {
                _left_speed = forward_speed; 
                _right_speed = backward_speed;
            }
            break;
        default:
            break;
        }
        // set the motor speeds
        _left_wheel_motor->setVelocity(_left_speed);
        _right_wheel_motor->setVelocity(_right_speed);
    }
}

void MyRobot::lock_state(int ticks)
{
    if (_state_timer == 0)
    {
        _state_timer = ticks;
    }
}

//////////////////////////////////////////////

double MyRobot::convert_bearing_to_degrees(const double *in_vector)
{
    double rad = atan2(in_vector[2], in_vector[0]);
    double deg = rad * (180.0 / M_PI);
    if (deg < 0.0)
    {
        deg += 360.0;
    }
    if (deg > 357.0 || deg < 3.0)
    {
        return 0.0;
    }

    return deg;
}

//////////////////////////////////////////////
