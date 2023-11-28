#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : Robot()
{
    // init default values
    int timeStep = (int)robot->getBasicTimeStep();

    // Variables odometria
    _x = _y = _theta = _x_ant = _y_ant = _theta_ant = ir_frontal = 0.0; // robot pose variables
    _sr = _sl = _sl_ant = _sr_ant = 0.0;                                // displacement right and left wheels
    _x_goal = 16.4, _y_goal = 0,
    _theta_goal = atan2((_y_goal - _y), (_x_goal - _x)); // target pose

    // Variables de velocidad =0 al comienzo
    _left_speed = 0;
    _right_speed = 0;

    // set mode to fordward
    _posicion = LIBRE;

    // get and enable the compass device
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);

    // Iniciar Sensores de Distancia
    _distance_sensor[0] = getDistanceSensor("ds0");
    _distance_sensor[0]->enable(_time_step);
    _distance_sensor[1] = getDistanceSensor("ds3");
    _distance_sensor[1]->enable(_time_step);
    _distance_sensor[2] = getDistanceSensor("ds13");
    _distance_sensor[2]->enable(_time_step);

    // Iniciar Encoders
    _left_wheel_sensor = getPositionSensor("left wheel sensor");
    _right_wheel_sensor = getPositionSensor("right wheel sensor");
    _left_wheel_sensor->enable(_time_step);
    _right_wheel_sensor->enable(_time_step);

    // Iniciar Camaras
    _forward_camera = getCamera("camera_f");
    _forward_camera->enable(_time_step);
    _spherical_camera = getCamera("camera_s");
    _spherical_camera->enable(_time_step);

    // Iniciar Motores
    _left_wheel_motor = getMotor("left wheel motor");
    _right_wheel_motor = getMotor("right wheel motor");
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    // disable devices

    _left_wheel_motor->setVelocity(0.0);
    _right_wheel_motor->setVelocity(0.0);

    _my_compass->disable();

    for (int i = 0; i < NUM_DISTANCE_SENSOR; i++)
    {
        _distance_sensor[i]->disable();
    }

    _left_wheel_sensor->disable();
    _right_wheel_sensor->disable();
}

//////////////////////////////////////////////

void MyRobot::run()
{
    double compass_angle;
    double ir_frontal = 0.0, ir_izq = 0.0, ir_der = 0.0;
    int i = 0;
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
        cout << " Ir_der : " << ir_der << "Ir_frontal: " << ir_frontal << " Ir_izq: " << ir_izq << endl;

        if (ir_frontal > DISTANCE_LIMIT && ir_der > DISTANCE_LIMIT && ir_izq > DISTANCE_LIMIT)
        {
            cout << "BLOQUEO" << endl;
            _posicion = BLOQUEO;
        }
        if (ir_frontal > DISTANCE_LIMIT && ir_der < DISTANCE_LIMIT && ir_izq > DISTANCE_LIMIT)
        {
            cout << "ESQ_DERERECHA_LIBRE" << endl;
            _posicion = ESQ_DER;
        }
        if (ir_frontal < DISTANCE_LIMIT && ir_der < DISTANCE_LIMIT && ir_izq > DISTANCE_LIMIT)
        {
            cout << "PAR_IZQUIERDA" << endl;
            _posicion = PAR_IZ;
            pared_der = 0;
            pared_iz = 1;
        }
        if (ir_frontal < DISTANCE_LIMIT && ir_der < DISTANCE_LIMIT && ir_izq < DISTANCE_LIMIT)
        {
            cout << "LIBRE" << endl;
            _posicion = LIBRE;
        }
        if (ir_frontal < DISTANCE_LIMIT && ir_der > DISTANCE_LIMIT && ir_izq < DISTANCE_LIMIT)
        {
            cout << "PAR_DER" << endl;

            _posicion = PAR_DER;
            pared_der = 1;
            pared_iz = 0;
        }
        if (ir_frontal > DISTANCE_LIMIT && ir_der > DISTANCE_LIMIT && ir_izq < DISTANCE_LIMIT)
        {
            cout << "ESQ_IZ" << endl;

            _posicion = ESQ_IZ;
        }
        if (ir_frontal > DISTANCE_LIMIT && ir_der < DISTANCE_LIMIT && ir_izq < DISTANCE_LIMIT)
        {
            cout << "PAR_FRONT" << endl;

            _posicion = PAR_FRONT;
        }

        switch (_posicion)
        {
        case BLOQUEO:
            turn_left();
            break;

        case PAR_FRONT:
            turn_right();

            break;

        case ESQ_DER:
            turn_right();
            break;

        case PAR_IZ:
            forward();
            break;

        case LIBRE:
            if (pared_der == 1)
            {
                turn_right();
            }
            else if (pared_iz == 1)
            {
                turn_left();
            }
            else
            {

                forward();
            }
            break;

        case PAR_DER:
            forward();
            break;

        case ESQ_IZ:
            turn_left();
            break;

        default:
            cout << "ERROR DE CONTEMPACION" << endl;
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

/////////////////////////////
// METODOS DE CALCULO

double MyRobot::convert_bearing_to_degrees(const double *in_vector)
{
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);

    return deg;
}

void MyRobot::get_info()
{
}

/////////////////////////////////

//////////////////////////
// METODOS DE MOVIMIENTO

void MyRobot::forward()
{
    _left_speed = MAX_SPEED;
    _right_speed = MAX_SPEED;
}

void MyRobot::turn_left()
{
    cout << "Turn left" << endl;
    _left_speed = MAX_SPEED - 6;
    _right_speed = MAX_SPEED;
}
void MyRobot::turn_right()
{
    cout << "Turn right" << endl;
    _left_speed = MAX_SPEED;
    _right_speed = MAX_SPEED - 6;
}

void MyRobot::set_speed()
{
    // set the motor speeds
    _left_wheel_motor->setVelocity(_left_speed);
    _right_wheel_motor->setVelocity(_right_speed);
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

///////////////////////////////////////////

// METODOS ODOMETRIA
////////////////////////////////////////////////////////

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

double MyRobot::convert_bearing_to_degrees()
{
    const double *in_vector = _my_compass->getValues();

    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);

    return deg;
}

float MyRobot::encoder_tics_to_meters(float tics)
{
    return tics / ENCODER_TICS_PER_RADIAN * WHEEL_RADIUS;
}

void MyRobot::print_odometry()
{
    cout << "x:" << _x << " y:" << _y << " theta:" << _theta << endl;
    cout << _x_goal << endl;
}

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

double MyRobot::convert_bearing_to_degrees2(const double *in_vector)
{
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);

    return deg;
}
/////////////////////////////////////////////////
