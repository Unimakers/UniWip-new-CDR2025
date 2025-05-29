#include <constante.h>
#include <AccelStepper.h>
#include <lidar.h>
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
    Etat_mvt etat_ec=Etat_mvt::STILL;

    int vitesse_mT;
    AccelStepper
        stepper_gauche,
        stepper_droit;
    struct CurAction {int left, right;};
    CurAction currentaction;
    bool paused=false;
    long starttime=0;
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
    void run(bool *lidar);
    bool reachedtarget();
    void moveTo(Coord coord, int vitesse);
    bool moveToLoop();
    void forward(double distance, int vitesse);
    void backward(double distance, int vitesse);
    void turnTo(double angle, int vitesse);
    void turn(double angle, int vitesse);
    void setCoord(Coord startCoord);
    void debugPosition();
    void stop();
    void resume();
};