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
#define DESIRED_ANGLE   0
#define NUM_DISTANCE_SENSOR 16
#define DISTANCIA_CHOQUE 950

        
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
         * @brief Function for converting bearing vector from compass to angle (in degrees).
         * @param in_vector Input vector with the values to convert.
         * @return The value in degrees.
         */
        double convert_bearing_to_degrees(const double* in_vector);

    private:
        
        const double _infinity = numeric_limits<double>::infinity();
        // The time step
        int _time_step;
        
        // velocities
        double _left_speed, _right_speed;

        // Sensors
        Compass *_my_compass;
        DistanceSensor *_distance_sensor[NUM_DISTANCE_SENSOR];
        
        // Motors
        Motor *_left_wheel_motor;
        Motor *_right_wheel_motor;
        
};

#endif

