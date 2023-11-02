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
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    // get and enable the compass device
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);
    
    
    _distance_sensor[0]= getDistanceSensor("ds0");
    _distance_sensor[0]-> enable(_time_step);
    _distance_sensor[1]= getDistanceSensor("ds3");
    _distance_sensor[1]-> enable(_time_step);
    _distance_sensor[2]= getDistanceSensor("ds13");
    _distance_sensor[2]-> enable(_time_step);        
    
    
    _left_wheel_motor = getMotor("left wheel motor");
    _right_wheel_motor = getMotor("right wheel motor");
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    // disable devices
    _my_compass->disable();
    
    for (int i=0; i<NUM_DISTANCE_SENSOR; i++) {
      _distance_sensor[i]->disable();}


}

//////////////////////////////////////////////

void MyRobot::run()
{
    double compass_angle;
    double ir_frontal =0.0, ir_izq=0.0,ir_der=0.0;


    while (step(_time_step) != -1) {
        // read the sensors
        const double *compass_val = _my_compass->getValues();

        // convert compass bearing vector to angle, in degrees
        compass_angle = convert_bearing_to_degrees(compass_val);

        // print sensor values to console
        cout << "Compass angle (degrees): " << compass_angle << endl;
        
        
        //Sensor de distancia
        ir_frontal=_distance_sensor[0]->getValue();
        ir_izq=_distance_sensor[1]->getValue();
        ir_der=_distance_sensor[2]->getValue();
        
        
        
        cout << "Ir_frontal: " << ir_frontal << " Ir_der: " << ir_der << " Ir_izq: " << ir_izq <<endl;

 


        // simple bang-bang control
        
        
        //Si se va a chocar de frente e auna pared: Girar a la derecha
        if (ir_frontal > DISTANCIA_CHOQUE || (ir_frontal > DISTANCIA_CHOQUE && ir_izq > DISTANCIA_CHOQUE)) {
            // turn right
            _left_speed = MAX_SPEED;
            _right_speed = MAX_SPEED - 5;
            cout << "Girando Derecha "   << endl;
        }
        if (ir_frontal > DISTANCIA_CHOQUE && ir_der > DISTANCIA_CHOQUE) {
            // turn right
            _left_speed = MAX_SPEED-5;
            _right_speed = MAX_SPEED;
            cout << "Girando Derecha "   << endl;
        }
        
        if (ir_frontal > DISTANCIA_CHOQUE && ir_der > DISTANCIA_CHOQUE && ir_izq > DISTANCIA_CHOQUE) {
            // turn right
            _left_speed = MAX_SPEED-5;
            _right_speed = MAX_SPEED-10;
            cout << "Girando Derecha "   << endl;
        }
        
            if(ir_frontal < DISTANCIA_CHOQUE){
                // move straight forward
                cout<<"Moving forward"<<endl;
                _left_speed = MAX_SPEED;
                _right_speed = MAX_SPEED;}
            
        

        // set the motor position to non-stop moving
        _left_wheel_motor->setPosition(_infinity);
        _right_wheel_motor->setPosition(_infinity);
        
        // set the motor speeds
        _left_wheel_motor->setVelocity(_left_speed);
        _right_wheel_motor->setVelocity(_right_speed);
    }
}

//////////////////////////////////////////////

double MyRobot::convert_bearing_to_degrees(const double* in_vector)
{
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);

    return deg;
}

//////////////////////////////////////////////

