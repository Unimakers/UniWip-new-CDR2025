#include <Robotmove.h>

RobotMove::RobotMove()
{
    this->stepper_droit = AccelStepper(AccelStepper::DRIVER, Pin::Driver::STEP_D, Pin::Driver::DIR_D);
    this->stepper_gauche = AccelStepper(AccelStepper::DRIVER, Pin::Driver::STEP_G, Pin::Driver::DIR_G);
    this->stepper_droit.setPinsInverted(1);
}
void RobotMove::setCoord(RobotMove::Coord startcoord)
{
    this->coordInst = startcoord;
}
void RobotMove::run()
{
    stepper_droit.run();
    stepper_gauche.run();
}

bool RobotMove::reachedtarget()
{
    if (not bool(bool(stepper_droit.distanceToGo()) + bool(stepper_gauche.distanceToGo())))
    {
        if (etat_ec != Etat_mvt::STILL)
        {
            return moveToLoop();
        }
        return true;
    }
    return false;
}

bool RobotMove::moveToLoop()
{
    if (etat_ec == Etat_mvt::TURN_S)
    {
        etat_ec = Etat_mvt::FORWARD;
        float d = sqrt(pow((destination.x - coordInst.x), 2) + pow((destination.y - coordInst.y), 2));
        forward(d, vitesse_mT);
        return false;
    }
    else if (etat_ec == Etat_mvt::FORWARD)
    {
        etat_ec = Etat_mvt::TURN_F;
        turnTo(destination.a, vitesse_mT);
        return false;
    }
    else
    {
        etat_ec = Etat_mvt::STILL;
        return true;
    }
}

void RobotMove::forward(int distance, int vitesse)
{
    int nbr_step = distance * Physique::STEP_CM;
    stepper_droit.setAcceleration(vitesse/2);
    stepper_droit.setMaxSpeed(vitesse);
    stepper_droit.move(distance * Physique::STEP_CM);

    stepper_gauche.setAcceleration(vitesse/2);
    stepper_gauche.setMaxSpeed(vitesse);
    stepper_gauche.move(distance * Physique::STEP_CM);
    Serial.print("set max speed");
    Serial.println(vitesse);

    coordInst.x += distance * sin(coordInst.a);
    coordInst.y += distance * cos(coordInst.a);
}

void RobotMove::backward(int distance, int vitesse)
{
    forward(-distance, vitesse);
}

void RobotMove::turn(double angle, int vitesse)
{
    stepper_droit.setAcceleration(Physique::ACCELARATION);
    stepper_droit.setMaxSpeed(vitesse * Physique::STEP_CM);

    stepper_gauche.setAcceleration(Physique::ACCELARATION);
    stepper_gauche.setMaxSpeed(vitesse * Physique::STEP_CM);

    int nbr_step = (Physique::ECRT_ROUE / 2) * (angle * PI / 180);

    stepper_droit.move(nbr_step * Physique::STEP_CM);
    stepper_gauche.move(-nbr_step * Physique::STEP_CM);

    coordInst.a += angle;
}

void RobotMove::turnTo(double angle, int vitesse)
{
    int angle_rel = coordInst.a - angle;
    if (angle_rel > 180)
    {
        turn(180 - angle_rel, vitesse);
    }
    else
    {
        turn(angle_rel, vitesse);
    }
}

void RobotMove::moveTo(Coord coord, int vitesse)
{
    turnTo(atan((coord.x - coordInst.x) / (coord.y - coordInst.y)), vitesse);
    etat_ec = Etat_mvt::TURN_S;
    destination = coord;
    vitesse_mT = vitesse;
}
void RobotMove::debugPosition(){
    Serial.print(stepper_gauche.currentPosition());
    Serial.print(";;");
    Serial.println(stepper_droit.currentPosition());
}