#include <Robotmove.h>

RobotMove::RobotMove(RobotMove::Coord startcoord){
    this->coordInst=startcoord;
    this->stepper_droit=AccelStepper(AccelStepper::DRIVER,Pin::Driver::STEP_D,Pin::Driver::DIR_D);    
    this->stepper_gauche=AccelStepper(AccelStepper::DRIVER,Pin::Driver::STEP_G,Pin::Driver::DIR_G);
}