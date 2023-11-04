#include <webots/Robot.hpp>
// Añadir un nuevo cabecero
#include <webots/Motor.hpp>
#include <math.h>

#define TIME_STEP 64
// Todas las clases de webots están definidas en el namespace “webots”
using namespace webots;
int main(int argc, char **argv) {
Robot *robot = new Robot();
// Obtener los parámetros de los motores
Motor *leftMotor = robot->getMotor("left wheel motor");
Motor *rightMotor = robot->getMotor("right wheel motor");
// Fijar el objetivo en posición de los motores

leftMotor->setPosition(INFINITY);
rightMotor->setPosition(INFINITY);

leftMotor->setVelocity(3);
rightMotor->setVelocity(5);

while (robot->step(TIME_STEP) != -1);
delete robot;
return 0;
}
