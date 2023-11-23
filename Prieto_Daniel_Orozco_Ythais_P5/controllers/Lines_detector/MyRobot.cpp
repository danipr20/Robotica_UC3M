/**
 * @file    MyRobot.cpp
 * @brief   Controller example for controlling the cameras of the robot.
 *
 * @author  Raul Perula-Martinez <raul.perula@uc3m.es>
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

    // get cameras and enable them
    _forward_camera = getCamera("camera_f");
    _forward_camera->enable(_time_step);
    _spherical_camera = getCamera("camera_s");
    _spherical_camera->enable(_time_step);


    // Motor initialization
    _left_wheel_motor = getMotor("left wheel motor");
    _right_wheel_motor = getMotor("right wheel motor");

    // Set position to infinity to allow velocity control
    _right_wheel_motor->setPosition(INFINITY); 
    _left_wheel_motor->setPosition(INFINITY);

    // Set initial velocity to 0
    _right_wheel_motor->setVelocity(0.0);
    _left_wheel_motor->setVelocity(0.0);
}




//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    // disable camera devices
    _forward_camera->disable();
    _spherical_camera->disable();
}

//////////////////////////////////////////////

void MyRobot::run()
{   bool final=false;
    int sum_f = 0;
    int sum_es=0;
        unsigned char green = 0, red = 0, blue = 0;
         unsigned char green_ = 0, red_ = 0, blue_ = 0;
    double percentage_white = 0.0;
        double percentage_yellow = 0.0;

int a=1;
    // get size of images for forward camera
    int image_width_f = _forward_camera->getWidth();
    int image_height_f = _forward_camera->getHeight();
    cout << "Size of forward camera image: " << image_width_f << ", " <<  image_height_f << endl;

    // get size of images for spherical camera
    int image_width_s = _spherical_camera->getWidth();
    int image_height_s = _spherical_camera->getHeight();
    cout << "Size of spherical camera image: " << image_width_s << ", " << image_height_s << endl;

    while (step(_time_step) != -1) {
        sum_f = 0;
                sum_es = 0;


        // get current image from forward camera
        const unsigned char *image_f = _forward_camera->getImage();
        const unsigned char *image_es = _spherical_camera->getImage();


        // count number of pixels that are white
        // (here assumed to have pixel value > 245 out of 255 for all color components)
        for (int x = 0; x < image_width_f; x++) {
            for (int y = 0; y < image_height_f; y++) {
                green_ = _forward_camera->imageGetGreen(image_f, image_width_f, x, y);
                red_ = _forward_camera->imageGetRed(image_f, image_width_f, x, y);
                blue_ = _forward_camera->imageGetBlue(image_f, image_width_f, x, y);

                if ((green_ > THRESHOLD) && (red_ > THRESHOLD) && (blue_ > THRESHOLD)) {
                    sum_f = sum_f + 1;
                }
            }
        }

       for (int x = 0; x < image_width_s; x++) {
            for (int y = 0; y < image_height_s; y++) {
                green = _spherical_camera->imageGetGreen(image_es, image_width_s, x, y);

                red = _spherical_camera->imageGetRed(image_es, image_width_s, x, y);

                blue = _spherical_camera->imageGetBlue(image_es, image_width_s, x, y);


                if ((green > ama) && (red > ama) && (blue < THRESHOLD)) {
                    sum_es = sum_es + 1;
                }
            }
        }

        percentage_white = (sum_f / (float) (image_width_f * image_height_f)) * 100;
        percentage_yellow = (sum_es / (float) (image_width_s * image_height_s)) * 100;

    


        //cout << "Percentage of white in forward camera image: " << percentage_white << endl;
        cout << "Percentage of yellow in forward camera image: " << percentage_yellow << endl;


        // movimiento aleatorio basado en el porcentaje de blanco

        if(percentage_white<90){
            _left_speed = 8;
            _right_speed = 10;
             a=a;
        }
         else if(percentage_white>95){
            _left_speed = 1;
            _right_speed = -1;
        }
       else{ _left_speed = 1;
            _right_speed = -2;}
        // set the motor speeds

        //detector de amarillo


        if(percentage_yellow>2.4){
            cout<< "linea"<< endl;
             final=true;
            
        }
        if (final==true){
            _left_speed = 0;
            _right_speed = 0;
        }
            
        _right_wheel_motor->setVelocity(_right_speed);
        _left_wheel_motor->setVelocity(_left_speed);
    }
}

//////////////////////////////////////////////
