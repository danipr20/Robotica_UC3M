#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : Robot()
{
    // init default values
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;
    // set mode to fordward
    _mode = FORWARD;
    // get and enable the compass device
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);

    _distance_sensor[0] = getDistanceSensor("ds0");
    _distance_sensor[0]->enable(_time_step);
    _distance_sensor[1] = getDistanceSensor("ds3");
    _distance_sensor[1]->enable(_time_step);
    _distance_sensor[2] = getDistanceSensor("ds13");
    _distance_sensor[2]->enable(_time_step);

    _left_wheel_motor = getMotor("left wheel motor");
    _right_wheel_motor = getMotor("right wheel motor");
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    // disable devices
    _my_compass->disable();

    for (int i = 0; i < NUM_DISTANCE_SENSOR; i++)
    {
        _distance_sensor[i]->disable();
    }
}

//////////////////////////////////////////////

void MyRobot::run()
{
    int modo;
    double compass_angle;
    double ir_frontal = 0.0, ir_izq = 0.0, ir_der = 0.0;

    bool pared_iz = 0;
    bool pared_der = 0;
    // set the motor position to non-stop moving
    _left_wheel_motor->setPosition(_infinity);
    _right_wheel_motor->setPosition(_infinity);

    while (step(_time_step) != -1)
    {
        const double *compass_val = _my_compass->getValues();

        compass_angle = convert_bearing_to_degrees(compass_val);
        // convert compass bearing vector to angle, in degrees
        ir_frontal = _distance_sensor[0]->getValue();
        ir_izq = _distance_sensor[1]->getValue();
        ir_der = _distance_sensor[2]->getValue();
        get_info();
        cout << "Ir_frontal: " << ir_frontal << " Ir_der: " << ir_der << " Ir_izq: " << ir_izq << endl;

        if (ir_frontal > DISTANCE_LIMIT && ir_der > DISTANCE_LIMIT && ir_izq > DISTANCE_LIMIT)
        {
            modo = BLOQUEO;
        }
        if (ir_frontal < DISTANCE_LIMIT)
        {
            modo = FORWARD;
        }

        switch (_mode)
        {
        case BLOQUEO:
            for (int i = 0; i < 100; i++)
            {
                backward();
            }
            for (int i = 0; i < 100; i++)
            {
                turn_left();
                set_speed();
            }

            break;
        case FORWARD:
            forward();
        default:
            break;
        }
        /*  if ((ir_der < DISTANCIA_CHOQUE && ir_frontal < DISTANCIA_CHOQUE && ir_izq < DISTANCIA_CHOQUE) || (ir_izq > DISTANCIA_CHOQUE && ir_frontal < DISTANCIA_CHOQUE))
          {
              // movimiento de frente, ninguna pared cerca o pared a la izquierda
              cout << "Moving forward" << endl;
              _left_speed = MAX_SPEED;
              _right_speed = MAX_SPEED;
          }
          if (ir_izq > DISTANCIA_CHOQUE)
          {
              pared_iz = 1;
              pared_der = 0;
          }
          if (ir_der > DISTANCIA_CHOQUE)
          {
              pared_der = 1;
              pared_iz = 0;
          }

          if (ir_izq < DISTANCIA_CHOQUE && pared_iz == 1)
          {
              // esquina hacia la izquierda, antes detectaba pared y ahora no
          }

          if ((ir_der > DISTANCIA_CHOQUE) || (ir_frontal > DISTANCIA_CHOQUE) || (ir_izq > DISTANCIA_CHOQUE && ir_frontal > DISTANCIA_CHOQUE))
          {
              // gira a la derecha
              cout << "Turn right" << endl;
              _left_speed = MAX_SPEED - 9;
              _right_speed = MAX_SPEED - 11;
          }*/

        // set the motor speeds
        set_speed();
    }
}

//////////////////////////////////////////////

double MyRobot::convert_bearing_to_degrees(const double *in_vector)
{
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);

    return deg;
}

//////////////////////////////////////////////
void MyRobot::forward(int i)
{
    _left_speed = MAX_SPEED;
    _right_speed = MAX_SPEED;
    for (int z = 0; z < i; z++)
    {
        set_speed();
    }
}
void MyRobot::set_speed(int i)
{
    // set the motor speeds
    _left_wheel_motor->setVelocity(_left_speed);
    _right_wheel_motor->setVelocity(_right_speed);
}
void MyRobot::turn_left(int i)
{
    cout << "Turn left" << endl;
    _left_speed = MAX_SPEED - 8;
    _right_speed = MAX_SPEED - 6;
    for (int z = 0; z < i; z++)
    {
        set_speed();
    }
}
void MyRobot::turn_right()
{
    cout << "Turn right" << endl;
    _left_speed = MAX_SPEED - 6;
    _right_speed = MAX_SPEED - 8;
    for (int z = 0; z < i; z++)
    {
        set_speed();
    }
}

void MyRobot::get_info()
{
}

void MyRobot::stop()
{
    _left_speed = 0;
    _right_speed = 0;
}
void MyRobot::esquivar()
{
    turn_right();
    seguir_contorno();
}
void MyRobot::seguir_contorno()
{
}
void MyRobot::desbloquear()
{
    void backward();
}
void MyRobot::backward()
{
    _left_speed = -MAX_SPEED;
    _right_speed = -MAX_SPEED;
}
