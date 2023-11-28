/**
 * @file    MyRobot.cpp
 * @brief   A simple example for computing the odometry while the robot moves straight
 *
 * @author  Raul Perula-Martinez <raul.perula@uc3m.es>
 * @author  Alvaro Castro-Gonzalez <acgonzal@ing.uc3m.es>
 * @author  Javier Pastor Fernandez <javpasto@ing.uc3m.es>
 * @date    2021-12
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : Robot()
{

  // init default values
  _time_step = 64;

  _left_speed = 0;
  _right_speed = 0;

  _x = _y = _theta = _x_ant = _y_ant = _theta_ant = ir_frontal = 0.0; // robot pose variables
  _sr = _sl = _sl_ant = _sr_ant = 0.0;                                // displacement right and left wheels
  _x_goal = 16.4, _y_goal = 0,
  _theta_goal = atan2((_y_goal - _y), (_x_goal - _x)); // target pose

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

  // Set motor position to 0 to re-initialize the encoder measurement
  _right_wheel_motor->setPosition(0.0);
  _left_wheel_motor->setPosition(0.0);

  // Set motor position to infinity to allow velocity control
  _right_wheel_motor->setPosition(INFINITY);
  _left_wheel_motor->setPosition(INFINITY);

  // Set motor velocity to 0
  _right_wheel_motor->setVelocity(0.0);
  _left_wheel_motor->setVelocity(0.0);

  _distance_sensor[0] = getDistanceSensor("ds0");
  _distance_sensor[0]->enable(_time_step);
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
  // Stop motors
  _left_wheel_motor->setVelocity(0.0);
  _right_wheel_motor->setVelocity(0.0);

  // Disable robot's sensors
  _my_compass->disable();
  _left_wheel_sensor->disable();
  _right_wheel_sensor->disable();

  _distance_sensor[0]->disable();
}

//////////////////////////////////////////////
// Controller main logic
void MyRobot::run()
{
  cout << "Goal --> x: " << _x_goal << endl;
  cout << "Goal --> y: " << _y_goal << endl;
  cout << "Goal --> Theta: " << _theta_goal << endl;
  cout << "Inicio ir frontal= " << ir_frontal << endl;

  _left_speed = MAX_SPEED;
  _right_speed = MAX_SPEED;

  // set the motor speeds
  _left_wheel_motor->setVelocity(_left_speed);
  _right_wheel_motor->setVelocity(_right_speed);

  while (step(_time_step) != -1)
  {
    ir_frontal = _distance_sensor[0]->getValue();
    const double *compass_val = _my_compass->getValues();

    // convert compass bearing vector to angle, in degrees
    compass_angle = convert_bearing_to_degrees2(compass_val);
    this->compute_odometry();
    this->print_odometry();

    // print sensor values to console
    cout << "Compass angle (degrees): " << compass_angle << endl
         << "IR_FRONTAL=" << ir_frontal << endl;

    _sl = encoder_tics_to_meters(_left_wheel_sensor->getValue());
    _sr = encoder_tics_to_meters(_right_wheel_sensor->getValue());

    cout << "Left encoder: " << _sl << endl;
    cout << "Right encoder: " << _sr << endl;

    if (ir_frontal > 200)
    {
      cout << "Modo giro TRUE " << endl;

      _left_speed = MAX_SPEED - 9;
      _right_speed = MAX_SPEED - 11;
    }

    if (ir_frontal < 200)
    {

      _left_speed = MAX_SPEED;
      _right_speed = MAX_SPEED;
    }

    _left_wheel_motor->setVelocity(_left_speed);
    _right_wheel_motor->setVelocity(_right_speed);

    this->goal_reached();
  }
}

//////////////////////////////////////////////
void MyRobot::compute_odometry()
{
  _inc_sl = _sl - _sl_ant;
  _inc_sr = _sr - _sr_ant;

  _theta = _theta_ant + (_inc_sr - _inc_sl) / WHEELS_DISTANCE;
  _x = _x_ant + ((_inc_sr + _inc_sl) / (2) * cos(_theta + ((_inc_sr - _inc_sl) / (2 * WHEELS_DISTANCE))));
  _y = _y_ant + ((_inc_sr + _inc_sl) / (2) * sin(_theta + ((_inc_sr - _inc_sl) / (2 * WHEELS_DISTANCE))));

  _x_ant = _x;
  _y_ant = _y;
  _theta_ant = _theta;

  _sl_ant = _sl;
  _sr_ant = _sr;
}
//////////////////////////////////////////////

double MyRobot::convert_bearing_to_degrees()
{
  const double *in_vector = _my_compass->getValues();

  double rad = atan2(in_vector[0], in_vector[2]);
  double deg = rad * (180.0 / M_PI);

  return deg;
}
//////////////////////////////////////////////

float MyRobot::encoder_tics_to_meters(float tics)
{
  return tics / ENCODER_TICS_PER_RADIAN * WHEEL_RADIUS;
}

//////////////////////////////////////////////

void MyRobot::print_odometry()
{
  cout << "x:" << _x << " y:" << _y << " theta:" << _theta << endl;
  cout << _x_goal << endl;
}

//////////////////////////////////////////////

bool MyRobot::goal_reached()
{
  if (_x > _x_goal)
  {
    _left_wheel_motor->setVelocity(0);
    _right_wheel_motor->setVelocity(0);
    cout << "Mu bien majo" << endl
         << endl;
    return true;
  }
  return false;
}
//////////////////////////////////////////////

double MyRobot::convert_bearing_to_degrees2(const double *in_vector)
{
  double rad = atan2(in_vector[0], in_vector[2]);
  double deg = rad * (180.0 / M_PI);

  return deg;
}