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
    
    _x = _y = _theta = 0.0; // robot pose variables
    _sr = _sl = 0.0;        // displacement right and left wheels

    _backing_direction = true;
    // get distance sensor array and enable each one
    for (int i = 0; i < NUM_DISTANCE_SENSOR; i++)
    {
        _distance_sensor[i] = getDistanceSensor(ds_name[i]);
        _distance_sensor[i]->enable(_time_step);
    }

    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);
    
    // Motor Position Sensor initialization
    _left_wheel_sensor = getPositionSensor("left wheel sensor");
    _right_wheel_sensor = getPositionSensor("right wheel sensor");

    _left_wheel_sensor->enable(_time_step);
    _right_wheel_sensor->enable(_time_step);

    // Get robot's compass; initialize it
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);

    // Motor initialization
    _left_wheel_motor = getMotor("left wheel motor");
    _right_wheel_motor = getMotor("right wheel motor");

    // Get robot's GPS; initialize it
    _my_gps = getGPS("gps");
    _my_gps->enable(_time_step);

    // Set motor position to 0 to re-initialize the encoder measurement
    _right_wheel_motor->setPosition(0.0);
    _left_wheel_motor->setPosition(0.0);

    // Set motor position to infinity to allow velocity control
    _right_wheel_motor->setPosition(INFINITY);
    _left_wheel_motor->setPosition(INFINITY);

    // Set motor velocity to 0
    _right_wheel_motor->setVelocity(0.0);
    _left_wheel_motor->setVelocity(0.0);

    _front_cam = getCamera("camera_f");
    _front_cam->enable(_time_step);
    std::cout << "Front images are " << _front_cam->getWidth() << " x " << _front_cam->getHeight() << std::endl;
    _spher_cam = getCamera("camera_s");
    _spher_cam->enable(_time_step);
    std::cout << "Spherical images are " << _spher_cam->getWidth() << " x " << _spher_cam->getHeight() << std::endl;  
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    // stop the robot
    _right_wheel_motor->setVelocity(0.0);
    _left_wheel_motor->setVelocity(0.0);
    
    // disable devices
    for (int i = 0; i < NUM_DISTANCE_SENSOR; i++)
    {
        _distance_sensor[i]->disable();
    }
    _left_wheel_sensor->disable();
    _right_wheel_sensor->disable();
    _my_compass->disable();
    _front_cam->disable();
    _spher_cam->disable();
}

//////////////////////////////////////////////

void MyRobot::run() {

    double start_x, start_y;
    // get initial position via gps
    if(step(_time_step) != -1) {
        _x = _my_gps->getValues()[2];
        _y = _my_gps->getValues()[0];
        _x_goal = 18.0 + _x;
        _y_goal = _y;
        cout << "Goal: " << _x_goal << ", " << _y_goal << endl;
        start_x = _x;
        start_y = _y;
    }
    go_to_point(_x_goal, _y_goal);
    cout << "Goal Reached, heading towards start: (" << start_x << ", " << start_y << ")" << endl;
    
    // ====NOTE: Find green pillars here====

    go_to_point(start_x, start_y);
    cout << "Start Reached, GPS: (" << _my_gps->getValues()[2] << ", " << _my_gps->getValues()[0] << ")" << endl;
}

void MyRobot::go_to_point(double x, double y)
{
    _x_goal = x;
    _y_goal = y;

    // double ir1_val = 0.0, ir14_val = 0.0;
    double back_right = 0.0, front_right = 0.0; // initialize distance sensor values
    double back_left = 0.0, front_left = 0.0;
    double compass_angle = 0.0;

    while (step(_time_step) != -1 && !this->goal_reached())
    {
        // update location via odometry
        this->compute_odometry();
        if (verbose) {
            this->print_odometry();
        }
        // check if we are close to goal
        // if(this->goal_reached())
        //     cout << "Goal Reached";
        //     while(true);
        //     break;
        
        update_theta_goal();

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
        if (verbose) {
            cout << "angle goal: " << _theta_goal << " current: " << compass_angle << endl;
        }
        compute_odometry();

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
            if(compass_angle > _theta_goal - 120 && compass_angle < _theta_goal - 60 && back_left == 0 && front_left == 0) {
                new_mode = ORIENT;
            }
            if(compass_angle < _theta_goal + 120 && compass_angle > _theta_goal + 60 && back_right == 0 && front_right == 0) {
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
            if(compass_angle > _theta_goal - 2.0 && compass_angle < _theta_goal + 2.0) {
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
        double mult_factor = min(1.0, abs(compass_angle - _theta_goal) / 45.0);

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

            if (compass_angle <= _theta_goal - 2.0)
            {
                _left_speed = backward_speed;
                _right_speed = forward_speed;
            }
            if (compass_angle >= _theta_goal + 2.0)
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

//////////////////////////////////////////////

void MyRobot::compute_odometry()
{
    double newleft = (this->_left_wheel_sensor->getValue());
    double newright = (this->_right_wheel_sensor->getValue());
    newleft = encoder_tics_to_meters(newleft);
    newright = encoder_tics_to_meters(newright);

    // get deltas between previous and current reading
    double dsl = newleft - _sl;
    double dsr = newright - _sr;
    // update left/right encoder values
    _sl = newleft;
    _sr = newright;

    // decompose odometry formula into more convenient variables
    double dist_forward = (dsl + dsr) / 2;
    double angle_change = (dsr - dsl) / WHEELS_DISTANCE;

    // update odometry
    _x += dist_forward * cos(_theta + angle_change / 2);
    _y += dist_forward * sin(_theta + angle_change / 2);
    _theta += angle_change;
}

//////////////////////////////////////////////

float MyRobot::encoder_tics_to_meters(float tics)
{
    return tics / ENCODER_TICS_PER_RADIAN * WHEEL_RADIUS;
}

//////////////////////////////////////////////

void MyRobot::print_odometry()
{
    cout << "x:" << _x << " y:" << _y << endl;
}

//////////////////////////////////////////////

bool MyRobot::goal_reached()
{
    const float DIST_ERROR = 0.05;
    float x_error = abs(_x - _x_goal);
    float y_error = abs(_y - _y_goal);
    return x_error <= DIST_ERROR && y_error <= DIST_ERROR;
    // return false;
}
//////////////////////////////////////////////

void MyRobot::update_theta_goal()
{
    double rads_goal = atan2((_y_goal - _y), (_x_goal - _x)); // target pose
    double deg = rads_goal * (180.0 / M_PI);
    // if (deg < 0.0)
    // {
    //     deg += 360.0;
    // }
    deg += 180.0;
    if (deg > 357.0 || deg < 3.0)
    {
        deg = 0.0;
    }
    _theta_goal = deg;
}
