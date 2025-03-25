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
    verbose = false;
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
    // double ir1_val = 0.0, ir14_val = 0.0;
    double back_right = 0.0, front_right = 0.0; // initialize distance sensor values
    double back_left = 0.0, front_left = 0.0;
    double compass_angle = 0.0;
    while (step(_time_step) != -1)
    {
        // read the sensors
        back_right = _distance_sensor[2]->getValue();
        front_right = _distance_sensor[3]->getValue();
        front_left = _distance_sensor[4]->getValue();
        back_left = _distance_sensor[5]->getValue();
        double front_avg = (_distance_sensor[0]->getValue() + _distance_sensor[1]->getValue()) * 0.5;
        if (verbose) {
            cout << "l: " << _distance_sensor[0]->getValue() << " r: " << _distance_sensor[1]->getValue() << " front_avg: " << front_avg << endl;
        }

        const double *compass_val = _my_compass->getValues();
        compass_angle = convert_bearing_to_degrees(compass_val);

        // ====STATE TRANSITIONS====
        RobotMode new_mode = _mode;
        switch(_mode) {
        case FORWARD:
            if (_distance_sensor[0]->getValue() > SHORT_LIMIT || _distance_sensor[1]->getValue() > SHORT_LIMIT) {
                new_mode = OBSTACLE_AVOID;
                _backing_direction = !_backing_direction;
            }
        break;
        case OBSTACLE_AVOID:
            if(front_avg == 0) {
                new_mode = WALL_FOLLOW;
            }
        break;
        case WALL_FOLLOW:
            // break off from wall when free
            if(compass_angle > 60 && compass_angle < 120 && back_left == 0 && front_left == 0) {
                new_mode = ORIENT;
            }
            if(compass_angle < 300 && compass_angle > 240 && back_right == 0 && front_right == 0) {
                new_mode = ORIENT;
            }
            // turn interior corner
            if(front_avg > SHORT_LIMIT) {
                if(back_right > DISTANCE_LIMIT || front_right > DISTANCE_LIMIT ) {
                    new_mode = TURN_LEFT;
                }
                if(back_left > DISTANCE_LIMIT || front_left > DISTANCE_LIMIT ) {
                    new_mode = TURN_RIGHT;
                }
            }
        break;
        case TURN_LEFT:
        case TURN_RIGHT:
            if(front_avg == 0) {
                new_mode = WALL_FOLLOW;
            }
        break;
        case ORIENT: 
            if(compass_angle > 178 && compass_angle < 182) {
                new_mode = FORWARD;
            }
        break;
        default:
            new_mode = ORIENT;
        break;
        }

        if(_mode != new_mode) {
            _mode = new_mode;
            print_mode();
        }

        // turn speeds
        double forward_speed = MAX_SPEED / 4.0;
        double backward_speed = -MAX_SPEED / 5.0;
        double mult_factor = min(1.0, abs(compass_angle - 180.0) / 45.0);

        const double FOLLOW_SPEED = MAX_SPEED * 0.3;
        const double SLOW_FACTOR = 0.5;
        double speed_percent = max(0.1, (500 - front_avg) * 0.002);
        // send actuators commands according to the mode
        switch (_mode)
        {
        case FORWARD:
            // cout << "Moving forward." << endl;
            _left_speed = MAX_SPEED * speed_percent;
            _right_speed = MAX_SPEED * speed_percent;
            break;
        case OBSTACLE_AVOID:
            if (_backing_direction)
            {
                // cout << "Backing up and turning left." << endl;
                _left_speed = -MAX_SPEED / 6.0;
                _right_speed = MAX_SPEED / 6.0;
            }
            else
            {
                // cout << "Backing up and turning right." << endl;
                _right_speed = -MAX_SPEED / 6.0;
                _left_speed = MAX_SPEED / 6.0;
            }
            break;
        case WALL_FOLLOW:
            if(_backing_direction) { 
                // wall on left
                if(back_right > DISTANCE_LIMIT && back_right > front_right) {
                    _left_speed = FOLLOW_SPEED;
                    _right_speed = FOLLOW_SPEED * SLOW_FACTOR;
                } else {
                    _left_speed = FOLLOW_SPEED * SLOW_FACTOR;
                    _right_speed = FOLLOW_SPEED;
                }
            } else {
                if(front_left > DISTANCE_LIMIT && back_left < front_left) {
                    _left_speed = FOLLOW_SPEED;
                    _right_speed = FOLLOW_SPEED * SLOW_FACTOR;
                } else {
                    _left_speed = FOLLOW_SPEED * SLOW_FACTOR;
                    _right_speed = FOLLOW_SPEED;
                }
            }
            break;
        case TURN_LEFT:
            // cout << "turning left." << endl;
            _left_speed = -MAX_SPEED / 6.0;
            _right_speed = MAX_SPEED / 9.0;
            break;
        case TURN_RIGHT:
            // cout <<  "turning right." << endl;
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

void MyRobot::print_mode() {
    cout << mode_names[_mode];
    if(_mode == OBSTACLE_AVOID) {
        cout << ' ';
        if(_backing_direction) {
            cout << "left";
        } else {
            cout << "right";
        }
    }
    cout << endl;
};
