#include <Robotmove.h>

RobotMove::RobotMove(RobotMove::Coord startcoord){
    this->coordInst=startcoord;
    this->stepper_droit=AccelStepper(AccelStepper::DRIVER,Pin::Driver::STEP_D,Pin::Driver::DIR_D);    
    this->stepper_gauche=AccelStepper(AccelStepper::DRIVER,Pin::Driver::STEP_G,Pin::Driver::DIR_G);
    this->stepper_droit.setPinsInverted(1);
}

void RobotMove::forward(int distance,int vitesse){
    int nbr_step=distance*Physique::STEP_CM;
    stepper_droit.setAcceleration(Physique::ACCELARATION);
    stepper_droit.setSpeed(vitesse*Physique::STEP_CM);
    stepper_droit.move(distance*Physique::STEP_CM);

    stepper_gauche.setAcceleration(Physique::ACCELARATION);
    stepper_gauche.setSpeed(vitesse*Physique::STEP_CM);
    stepper_gauche.move(distance*Physique::STEP_CM);
}    

void RobotMove::backward(int distance,int vitesse){
    forward(-distance,vitesse);
}

void RobotMove::turn(double angle,int vitesse){
    stepper_droit.setAcceleration(Physique::ACCELARATION);
    stepper_droit.setSpeed(vitesse*Physique::STEP_CM);

    stepper_gauche.setAcceleration(Physique::ACCELARATION);
    stepper_gauche.setSpeed(vitesse*Physique::STEP_CM);

    int nbr_step=(Physique::ECRT_ROUE/2)*(angle*PI/180);

    stepper_droit.move(nbr_step*Physique::STEP_CM);
    stepper_gauche.move(-nbr_step*Physique::STEP_CM);
}

void RobotMove::turnTo(double angle,int vitesse){
    int angle_rel=coordInst.a-angle;
    if (angle_rel>180){
        turn(180-angle_rel,vitesse);
    }
    else{
        turn(angle_rel,vitesse);
    }
}

void RobotMove::moveTo(Coord coord,int vitesse){
    
}