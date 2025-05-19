#include <constante.h>
#include <AccelStepper.h>
class RobotMove
{
protected:

    enum struct Etat_mvt
    {
        TURN_S,
        FORWARD,
        TURN_F,
        STILL
    };
    Etat_mvt etat_ec;

    int vitesse_mT;
    AccelStepper
        stepper_gauche,
        stepper_droit;

public:
    struct Coord
    {
        double
            x,
            y,
            a;
    };
    Coord
        coordInst,
        destination;

    RobotMove();
    void run();
    bool reachedtarget();
    void moveTo(Coord coord, int vitesse);
    bool moveToLoop();
    void forward(int distance, int vitesse);
    void backward(int distance, int vitesse);

    void turnTo(double angle, int vitesse);
    void turn(double angle, int vitesse);
    void setCoord(Coord startCoord);
};