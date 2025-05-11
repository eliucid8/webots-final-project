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

    _passed_yellow_line = false;
    returning_rescue = false;
    victim_count = 0;
    
    in_cooldown = false;
    cooldown_counter = 0;
    performed_turnaround = false;

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
        _x_goal = 16.0 + _x;
        _y_goal = _y;
        cout << "Goal: " << _x_goal << ", " << _y_goal << endl;
        start_x = _x;
        start_y = _y;
    }

    go_to_point(_x_goal, _y_goal);
    cout << "Goal Reached: (" << _x << ", " << _y << ")" << endl;

    // find and rescue humans;
    rescue_procedure();
    // ====NOTE: Find green pillars here====
    // while (step(_time_step) != -1) {
    //     go_to_point(_x_goal, _y_goal);

    //     // add return home part *****
    // }
    // cout << "test" << endl;


    // all to be added to while above *****
    if (victim_count >= 2 && !returning_rescue) {
        cout << " All victims rescued. Switching to return mode." << endl;
        _x_goal = start_x;
        _y_goal = start_y;
        returning_rescue = true;
        cout << "All rescued, heading towards start: (" << start_x << ", " << start_y << ")" << endl;
    }
    // ** need to disregard points while in zone now UNTIL victim_rescued = 2
    // while (step(_time_step) != -1) {
    //     if (detect_victim()) {
    //         cout << "Green object detected!" << endl;
    //         break;
    //     }
    //     else {
    //         cout << "No green object detected yet." << endl;
    //     }
    // }
    // =====GREEN PILLAR CODE ABOVE
    go_to_point(start_x, start_y);
    cout << "Start Reached, GPS: (" << _my_gps->getValues()[2] << ", " << _my_gps->getValues()[0] << ")" << endl;
}

void MyRobot::rescue_procedure() {
    cout << "Starting rescue" << endl;

    while(step(_time_step) != -1) {
        cout << "Passed Yellow Line: " << _passed_yellow_line << endl;
        // yellow line exit turnaround test *****
        check_and_handle_turnaround();

        // Victim detection logic
        if (/*_passed_yellow_line && */victim_count < 2 && !in_cooldown) {
            if (detect_victim()) {
                cout << "victim bug" << endl;
                cout << "Victim green found " << endl;
                // move toward victim
                bug_to_victim();
                handle_victim_detection();  // spin, count victim, start cooldown
                //break;
            } else {
                _left_wheel_motor->setVelocity(-0.2 * MAX_SPEED);
                _right_wheel_motor->setVelocity(0.2 * MAX_SPEED);
            }
        }
        if (in_cooldown) {
            cooldown_counter++;
            if (cooldown_counter >= COOLDOWN_STEPS) {
                in_cooldown = false;
                cooldown_counter = 0;
                cout << "Cooldown complete. Victim detection re-enabled." << endl;
            }
        }
    
        // // ++++ Yellow line detection --> zone transition
        // if (!_passed_yellow_line && detect_yellow_line()) {
        //     cout << "Yellow line in SIGHT — switching to YELLOW_CROSS func" << endl;
        //         new_mode = YELLOW_CROSS;
        //         break;
        //     }
    }
}

void MyRobot::go_to_point(double x, double y)
{
    _x_goal = x;
    _y_goal = y;

    // double ir1_val = 0.0, ir14_val = 0.0;
    double back_right = 0.0, front_right = 0.0; // initialize distance sensor values
    double back_left = 0.0, front_left = 0.0;
    double compass_angle = 0.0;

    // original while commented for testing as goal is different (no coordinates)

    //while (step(_time_step) != -1 && !this->goal_reached())
    while (step(_time_step) != -1)
    {
        
        // update location via odometry
        this->compute_odometry();
        if (verbose) {
            this->print_odometry();
        }
        // check if we are close to goal
        if(this->goal_reached()) {
            _left_wheel_motor->setVelocity(0);
            _right_wheel_motor->setVelocity(0);
            break;
        }

        
        // if (!_passed_yellow_line && !returning_rescue) {
        //     update_theta_goal();
        // }
        
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

            // Original wall-follow transitions
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
            // if(compass_angle > _theta_goal - 2.0 && compass_angle < _theta_goal + 2.0) {
            //     new_mode = FORWARD;
            // }
            if (compass_angle > _theta_goal - 2.0 && compass_angle < _theta_goal + 2.0) {
                if (_passed_yellow_line) {
                    new_mode = WALL_FOLLOW;  // Stay in zone, resume following wall for victim search
                } else {
                    cout << "NO YELLOW" << endl;
                    new_mode = FORWARD;      // Still navigating toward goal
                }
            }
        break;

        break;

        // ++++++ added yellow line testing
        case YELLOW_CROSS:

            // Optional: go back to wall-follow once clear
            if (has_crossed_yellow_line()) {
                if (!_passed_yellow_line) {
                    _passed_yellow_line = true;
                    // TEMP speed values
                    //_left_speed = MAX_SPEED * 0.2;
                    //_right_speed = MAX_SPEED * 0.2;
                    cout << "Passed yellow goal line — now inside final zone." << endl;
                }
                new_mode = FORWARD;
                //new_mode = WALL_FOLLOW;
            }
            else {
                new_mode = YELLOW_CROSS;
            }
        break;
        // ++++++ added yellow line testing


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
            } else if (!_backing_direction){
                if(front_left > DISTANCE_LIMIT && back_left < front_left) {
                    _left_speed = FOLLOW_SPEED;
                    _right_speed = FOLLOW_SPEED * SLOW_FACTOR;
                } else {
                    _left_speed = FOLLOW_SPEED * SLOW_FACTOR;
                    _right_speed = FOLLOW_SPEED;
                }
            }
            else {
                cout << "error here" << endl;
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
        case YELLOW_CROSS:
            if (compass_angle > _theta_goal - 2.0 && compass_angle < _theta_goal + 2.0)
            {
                _left_speed = MAX_SPEED * 0.25;
                _right_speed = MAX_SPEED * 0.25;
            }
            else
            {
                // orient logic
                double forward_speed = MAX_SPEED * 0.2;
                double backward_speed = -MAX_SPEED * 0.2;
                double mult_factor = min(1.0, abs(compass_angle - _theta_goal) / 45.0);
                forward_speed *= mult_factor;
                backward_speed *= mult_factor;

                if (compass_angle <= _theta_goal - 2.0)
                {
                    _left_speed = backward_speed;
                    _right_speed = forward_speed;
                }
                else if (compass_angle >= _theta_goal + 2.0)
                {
                    _left_speed = forward_speed;
                    _right_speed = backward_speed;
                }
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
    std::cout << "x:" << _x << " y:" << _y << endl;
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

bool MyRobot::detect_yellow_line() {
    // const double* pos = _my_gps->getValues();
    // double z_pos = pos[2];

    // if (z_pos < 6.0) {
    //     return false;      // Too close, probably first line
    // }
    if (_x < _x_goal - 3.0) {
        return false;
    }

    const unsigned char* image = _front_cam->getImage();
    int width = _front_cam->getWidth();
    int height = _front_cam->getHeight();
    int yellow_pixel_count = 0;

    // Scan bottom third of the image where the floor is visible
    for (int y = height * 2 / 3; y < height; y += 2) {
        for (int x = 0; x < width; x += 2) {
            int r = _front_cam->imageGetRed(image, width, x, y);
            int g = _front_cam->imageGetGreen(image, width, x, y);
            int b = _front_cam->imageGetBlue(image, width, x, y);
            
            //std::cout << "red: " << r << " g:   " << g << endl;
            // (r-g) checks to see difference of red/green values is close as necesary for yellow
            if (r > 160 && g > 160 && b < 80 && abs(r - g) < 30) {
                yellow_pixel_count++;
                //std::cout << "count" << endl;

            }
        }
    }
    return  yellow_pixel_count > 50;
}

// for checking whole image
// bool MyRobot::detect_victim() {
//     const unsigned char* image = _front_cam->getImage();
//     int width = _front_cam->getWidth();
//     int height = _front_cam->getHeight();
//     int green_pixel_count = 0;

//     // loop through image rows, check every 2 pixels for performance optimisation
//     for (int y = 0; y < height; y +=2) {
//         for (int x = 0; x < width; x+= 2) {
//             int r = _front_cam->imageGetRed(image, width, x, y);
//             int g = _front_cam->imageGetGreen(image, width, x, y);
//             int b = _front_cam->imageGetBlue(image, width, x, y);

//             // check if green (g) is dominant component
//             // greater than 60 and at least 10 more than both red (r) and blue (b)
//             //std::cout << "Green: " << g << endl;
//             if (g > 60 && g > r + 10 && g > b + 10) {
//                 green_pixel_count++;
//         }
//     }
//    std::cout << "Green pixels detected: " << green_pixel_count << std::endl;

//     // bool will return true if green_pixel_count > 100 (threshold number)
//     return green_pixel_count > 30; 
// }
// }

// for center of image
bool MyRobot::detect_victim() {
    const unsigned char* image = _front_cam->getImage();
    int width = _front_cam->getWidth();
    int height = _front_cam->getHeight();
    int green_pixel_count = 0;

    // Focus on center vertical stripe
    int center_x_min = width / 3;
    int center_x_max = 2 * width / 3;

    for (int y = 0; y < height; y += 2) {
        for (int x = center_x_min; x < center_x_max; x += 2) {
            int r = _front_cam->imageGetRed(image, width, x, y);
            int g = _front_cam->imageGetGreen(image, width, x, y);
            int b = _front_cam->imageGetBlue(image, width, x, y);

            if (g > 120 && g > r + 47 && g > b + 47) {
                green_pixel_count++;
            }
        }
    }

    //cout << "Green pixels (center): " << green_pixel_count << endl;

    return green_pixel_count > 120;  // threshold can be tuned
}

bool MyRobot::has_crossed_yellow_line() {
    const double* pos = _my_gps->getValues();
    double z_pos = pos[2];
    //std::cout << "Position → X: " << pos[0] << ", Y: " << pos[1] << ", Z: " << pos[2] << std::endl;
    if (z_pos > 8.0) {
        return true;
    }
    return false;
}


void MyRobot::handle_victim_detection() {

    in_cooldown = true;
    cooldown_counter = 0;

    std::cout << "Victim detected!" << std::endl;

    // Stop the robot before spinning
    _left_wheel_motor->setVelocity(0);
    _right_wheel_motor->setVelocity(0);
    step(_time_step);

    // rotate using compasss
    spin_in_place();

    // stop again after spin
    _left_wheel_motor->setVelocity(0);
    _right_wheel_motor->setVelocity(0);
    step(_time_step);

    // Update victim count
    victim_count++;
    cout << "Victim count: " << victim_count << " / 2" << endl;

    // DELAY before resuming --> to implement so that victim is not double counted
    for (int i = 0; i < 30; ++i) step(_time_step);
}

void MyRobot::bug_to_victim() {
    cout << "Approaching victim..." << endl;

    const double APPROACH_SPEED = MAX_SPEED * 0.2;  // slow, controlled approach
    const double STOP_DISTANCE = 500.0;             // stop when front sensors > this (tune as needed)
    const int MAX_APPROACH_STEPS = 100;             // timeout to prevent infinite loop

    int steps = 0;

    while (step(_time_step) != -1 && steps < MAX_APPROACH_STEPS) {
        steps++;

        // Check distance sensors 
        double front_left = _distance_sensor[3]->getValue();
        double front_right = _distance_sensor[4]->getValue();
        double front_avg = (front_left + front_right) * 0.5;
        cout << "Front avg: " << front_avg << endl;

        // Stop if within target distance
        if (front_avg < STOP_DISTANCE) {
            std::cout << "Victim within 1 meter — stopping approach." << std::endl;
            break;
        }

        // Optionally, stop if green is no longer detected
        if (!detect_victim()) {
            cout << "Lost sight of green object — abort approach." << endl;
            break;
        }
        // // Directional steering based on IR imbalance
        // double imbalance = front_right - front_left;

        // if (imbalance > 200.0) {
        //     // Victim skewed left → turn right 10°
        //     cout << " Victim skewed left — turning right 10°." << endl;
        //     _left_wheel_motor->setVelocity(0);
        //     _right_wheel_motor->setVelocity(0);
        //     turn_relative_angle(5.0);
        // }
        // else if (imbalance < -200.0) {
        //     // Victim skewed right → turn left 10°
        //     cout << " Victim skewed right — turning left 10°." << endl;     // ****testing line
        //     _left_wheel_motor->setVelocity(0);
        //     _right_wheel_motor->setVelocity(0);
        //     turn_relative_angle(-5.0);
        // }
         // Resume forward movement
        _left_wheel_motor->setVelocity(APPROACH_SPEED);
        _right_wheel_motor->setVelocity(APPROACH_SPEED);
        step(_time_step);  // take one step forward
    }

    // Final stop
    _left_wheel_motor->setVelocity(0);
    _right_wheel_motor->setVelocity(0);
    step(_time_step);
}


void MyRobot::spin_in_place() {
    double start_angle = convert_bearing_to_degrees(_my_compass->getValues());
    double total_rotation = 0.0;
    double last_angle = start_angle;

    const double SPIN_SPEED = MAX_SPEED * 0.3;

    for (int i = 0; i < 200; ++i) {
        _left_wheel_motor->setVelocity(-SPIN_SPEED);
        _right_wheel_motor->setVelocity(SPIN_SPEED);

        if (step(_time_step) == -1) break;

        double current_angle = convert_bearing_to_degrees(_my_compass->getValues());
        double delta = current_angle - last_angle;

        // Normalize for angle wraparound (e.g. 359 to 1 = +2° not -358°)
        if (delta < -180.0) delta += 360.0;
        if (delta > 180.0) delta -= 360.0;

        total_rotation += fabs(delta);
        last_angle = current_angle;

        // Stop after approximately one full spin
        if (total_rotation >= 360.0) break;
    }

    // Stop after spin
    _left_wheel_motor->setVelocity(0);
    _right_wheel_motor->setVelocity(0);
    step(_time_step);
}

void MyRobot::turn_relative_angle(double degrees) {
    double start_angle = convert_bearing_to_degrees(_my_compass->getValues());
    double target_angle = start_angle + degrees;

    // Normalize to [0, 360)
    if (target_angle >= 360.0) target_angle -= 360.0;
    if (target_angle < 0.0)    target_angle += 360.0;

    cout << "Turning from " << start_angle << "° to " << target_angle << "°" << endl;

     _left_wheel_motor->setVelocity(0);
    _right_wheel_motor->setVelocity(0);

    while (step(_time_step) != -1) {
        double current_angle = convert_bearing_to_degrees(_my_compass->getValues());

        // Calculate angular difference
        double error = target_angle - current_angle;
        if (error > 180)  error -= 360;
        if (error < -180) error += 360;

        // Stop if within 1° threshold
        if (abs(error) < 1.0) break;

        // Set turning speed based on direction
        double speed = MAX_SPEED * 0.2;
        if (error > 0) {
            _left_wheel_motor->setVelocity(speed);
            _right_wheel_motor->setVelocity(-speed);
        } else {
            _left_wheel_motor->setVelocity(-speed);
            _right_wheel_motor->setVelocity(speed);
        }
    }

    _left_wheel_motor->setVelocity(0);
    _right_wheel_motor->setVelocity(0);
    step(_time_step);
}

void MyRobot::check_and_handle_turnaround() {
    // const double* pos = _my_gps->getValues();
    // double z_pos = pos[2];

    if (_passed_yellow_line && !returning_rescue && _x < _x_goal /*z_pos < 7.0 */ && !performed_turnaround) {
        cout << "too far back check - turn 180, back to rescue zone" << endl;

        turn_relative_angle(180.0); 

        _theta_goal = convert_bearing_to_degrees(_my_compass->getValues());
        performed_turnaround = true;
    }
}
