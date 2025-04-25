#include <constante.h>
#include <AccelStepper.h>
class RobotMove
{
protected:
    typedef struct Coord
    {
        double
            x,
            y,
            a;
    };
    Coord coordInst;
    AccelStepper
        stepper_gauche,
        stepper_droit;

public:
    RobotMove(Coord startCoord);
    void moveTo(Coord coord,int vitesse); 
    void forward(int distance,int vitesse);
    void backward(int distance,int vitesse);
    void turnTo(double angle,int vitesse);
    void turn(double angle,int vitesse);
    
};