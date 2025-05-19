#include <Arduino.h>
#include <Robotmove.h>
#include <PCF8574.h>
#include <vector>

RobotMove robot;
enum struct etat
{
    PRE_INIT,
    INITALISATION,
    MATCH,
    FIN
};
etat etat_a = etat::PRE_INIT;
// PCF8574 pcf();

enum struct step_state
{
    RUNNING,
    IDLE,
    PAUSE
};

step_state etat_action;

enum struct atype
{
    FORWARD,
    BACKWARD,
    TURN,
    TURNTO,
    MOVETO
};

struct etape
{
    atype action;
    int distance;
    RobotMove::Coord coordonnees;
    double angle;
    int vitesse;
};

typedef std::vector<etape> strategie;

// etape{.action=atype::FORWARD,.distance=,.vitesse=},
// etape{.action=atype::TURN,.angle=,.vitesse=},
// etape{.action=atype::MOVETO,.coordonnees=,.vitesse=},

strategie stratun = strategie{
    etape{.action = atype::FORWARD, .distance = 100, .vitesse = 200},
    etape{.action = atype::TURN, .angle = 60, .vitesse = 200},
    etape{.action = atype::FORWARD, .distance = 100, .vitesse = 200},
    etape{.action = atype::TURN, .angle = 60, .vitesse = 200},
    etape{.action = atype::FORWARD, .distance = 100, .vitesse = 200},
    etape{.action = atype::TURN, .angle = 60, .vitesse = 200}};

strategie strat = stratun;

int etapeencour = 0;
void actioncall(etape step)
{
    switch (step.action)
    {
    case atype::FORWARD:
        return robot.forward(step.distance, step.vitesse);
        break;

    case atype::BACKWARD:
        return robot.backward(step.distance, step.vitesse);
        break;

    case atype::TURN:
        return robot.turn(step.angle, step.vitesse);

    case atype::TURNTO:
        return robot.turnTo(step.angle, step.vitesse);
        break;

    case atype::MOVETO:
        return robot.moveTo(step.coordonnees, step.vitesse);

    default:
        break;
    }
}

bool actionfini()
{
    return robot.reachedtarget();
}

void setup()
{
    Serial.begin(115200);
    pinMode(Pin::IHM::TIRETTE, INPUT_PULLUP);
    while(!digitalRead(Pin::IHM::TIRETTE)){
        Serial.println("what the fuck?");
        delay(100);
    }
    // fonction de pre_init
    while (digitalRead(Pin::IHM::TIRETTE))
    {
        Serial.println("Mais t'es pas là mais t'es où?");
        delay(100);
    }
    pinMode(Pin::Driver::EN, OUTPUT);
    digitalWrite(Pin::Driver::EN, LOW);
    etat_a = etat::INITALISATION;
}

void loop()
{
    if (etat_a != etat::MATCH)
    {
        if (etat_a == etat::FIN)
        {
            return;
        }
        if (not(digitalRead(Pin::IHM::TIRETTE)))
        {

            Serial.println("Jchuis là");
            // delay(100);
        }
        else
        {
            etat_a = etat::MATCH;
            Serial.println("fin setup");
            // delay(100);
        }
delay(100);
        return;
    }
    if (etat_action == step_state::RUNNING and actionfini())
    {
        etat_action = step_state::IDLE;
        // Serial.println("running one !");
    }
    else if (etat_action == step_state::RUNNING)
    {
        robot.run();
        // robot.debugPosition();
        // Serial.println("running two !");
    }
    else if (etat_action == step_state::IDLE)
    {
        etapeencour++;
        actioncall(strat[etapeencour]);
        etat_action = step_state::RUNNING;
        // Serial.println("running three !");
    }
    else{
        etat_action=step_state::IDLE;
        // Serial.println("running four !");
    }
    
}
