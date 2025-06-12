// TO SET STRATEGIE SEARCH STRATDEF
#include <Arduino.h>

/*
namespace Driver
{
    constexpr int
        DIR_G = D0, // 13,
        STEP_G = D1,
        DIR_D = D2,
        STEP_D = D3,
        EN = D8;
} // namespace Driver
#include <AccelStepper.h>
AccelStepper right;
void setup()
{
    pinMode(D8, OUTPUT);
    // pinMode(D10, INPUT_PULLUP);
    right = AccelStepper(AccelStepper::DRIVER, D3, D2);
    right.setMaxSpeed(8000);
    right.setAcceleration(4000);

    digitalWrite(D8, LOW);
    delay(1000);
    right.move(100000);
}
void loop()
{
    right.run();
}

*/

#include <constante.h>
#include <lidar.h>
#include <Robotmove.h>
#include <Adafruit_PCF8574.h>
#include <vector>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
// CODE NECESSAIRE AVANT DECLARATION DE STRATEGIE
enum struct atype
{
    FORWARD,
    BACKWARD,
    TURN,
    TURNTO,
    MOVETO,
    FERMER_AIMANTS,
    OUVRIR_AIMANTS,
    MONTER_ACTIONNEUR,
    DESCENDRE_ACTIONNEUR,
    MILLIEU_ACTIONNEUR,
    MONTER_BRAS,
    DESCENDRE_BRAS,
    ACTIVER_POMPE,
    DESACTIVER_POMPE,
    WAIT,
    MONTER_BANDEROLE,
    MONTER_CANETTE_2E_ETAGE,
    WAIT_END,
    ACTIONNEUR_POS,
    MILIEU_BRAS,
    ACTIVER_LIDAR,
    DESACTIVER_LIDAR
};
typedef atype A;
struct etape
{
    atype action;
    double distance;
    RobotMove::Coord coordonnees;
    double angle;
    int vitesse;
    int time;
};
constexpr int
    DEFAULT_SPEED =
#if PAMIMODE == 1
        2000
#else
        8000
#endif
    ;
etape FORWARD(double d, int v = DEFAULT_SPEED) { return etape{.action = A::FORWARD, .distance = d, .vitesse = v}; }
etape BACKWARD(double d, int v = DEFAULT_SPEED) { return etape{.action = A::BACKWARD, .distance = d, .vitesse = v}; }
etape TURN(double a, int v = DEFAULT_SPEED/2) { return etape{.action = A::TURN, .angle = a, .vitesse = v}; }
etape TURNTO(double a, int v = DEFAULT_SPEED) { return etape{.action = A::TURNTO, .angle = a, .vitesse = v}; }
etape MOVETO(RobotMove::Coord c, int v = DEFAULT_SPEED) { return etape{.action = A::MOVETO, .coordonnees = c, .vitesse = v}; }
etape FERMER_AIMANTS() { return etape{.action = A::FERMER_AIMANTS}; }
etape OUVRIR_AIMANTS() { return etape{.action = A::OUVRIR_AIMANTS}; }
etape MONTER_ACTIONNEUR() { return etape{.action = A::MONTER_ACTIONNEUR}; }
etape DESCENDRE_ACTIONNEUR() { return etape{.action = A::DESCENDRE_ACTIONNEUR}; }
etape MILLIEU_ACTIONNEUR() { return etape{.action = A::MILLIEU_ACTIONNEUR}; }
etape MONTER_BRAS() { return etape{.action = A::MONTER_BRAS}; }
etape DESCENDRE_BRAS() { return etape{.action = A::DESCENDRE_BRAS}; }
etape ACTIVER_POMPE() { return etape{.action = A::ACTIVER_POMPE}; }
etape DESACTIVER_POMPE() { return etape{.action = A::DESACTIVER_POMPE}; }
etape MONTER_BANDEROLE() { return etape{.action = A::MONTER_BANDEROLE}; }
etape WAIT(int time) { return etape{.action = A::WAIT, .time = time}; }
etape ACTIONNEUR_POS(double angle) { return etape{.action = A::ACTIONNEUR_POS, .angle = angle}; }
etape MONTER_CANNETTE_2E_ETAGE() { return etape{.action = A::MONTER_CANETTE_2E_ETAGE}; }
etape WAIT_END() { return etape{.action = A::WAIT_END}; }
etape MILIEU_BRAS(){return etape{.action=A::MILIEU_BRAS};}
typedef std::vector<etape> strategie;
Adafruit_PCF8574 pcf;

int PAMI_WAIT = 6000;

// DÉFINITION DE LA STRATÉGIE

/// @brief La stratégie numéro un du robot
strategie stratdemoservo = strategie{
    MILLIEU_ACTIONNEUR(),
    // WAIT(2000),
    // DESCENDRE_ACTIONNEUR(),
    // WAIT(1000),
    // OUVRIR_AIMANTS(),
    // WAIT(2000),
    // FERMER_AIMANTS(),
    // WAIT(1000),
    // MONTER_BRAS(),
    // WAIT(2000),
    // DESCENDRE_BRAS(),
};
strategie stratun = strategie{
    OUVRIR_AIMANTS(),
    WAIT(1000),
    FERMER_AIMANTS(),
    MILLIEU_ACTIONNEUR(),
    WAIT(1000),
    DESCENDRE_ACTIONNEUR(),
    FORWARD(40),
    // DESCENDRE_ACTIONNEUR(),
    // OUVRIR_AIMANTS(),
    // FORWARD(10),
    // MILLIEU_ACTIONNEUR(),
    // BACKWARD(10),
    // TURN(180),
    // FORWARD(30),
    // DESCENDRE_ACTIONNEUR(),
    // FERMER_AIMANTS()
};
// BETWEEN BACK AND WHEEL CENTER = 13.25cm
// BETWEEN BACK AND ACTIONNEUR AIMANT = 21cm
strategie noforfait = strategie{
    OUVRIR_AIMANTS(),
    WAIT(1000),
    FERMER_AIMANTS(),
    MILLIEU_ACTIONNEUR(),
    WAIT(1000),
    DESCENDRE_ACTIONNEUR(),
    FORWARD(40),
    BACKWARD(40),
};

// coord depart=1800/200
strategie stratapointblue = strategie{
    DESCENDRE_ACTIONNEUR(),
    OUVRIR_AIMANTS(),
    BACKWARD(30),
    TURN(-90),
    FORWARD(42.5),
    TURN(-90),
    BACKWARD(15),
    ACTIVER_POMPE(),
    WAIT(100),
    DESCENDRE_BRAS(),
    WAIT(500),
    MONTER_BRAS(),
    FORWARD(15),
    TURN(180,6000),
    FORWARD(40,4000),
    MONTER_CANNETTE_2E_ETAGE(),
    WAIT(1000),
    FORWARD(10,1000),
    BACKWARD(27.5),
    TURN(180,2000),
    BACKWARD(20),
    DESCENDRE_BRAS(),
    WAIT(500),
    DESACTIVER_POMPE(),
    WAIT(250),
    FORWARD(20),
    // fin premiere pile
    ACTIVER_POMPE(),
    MONTER_BRAS(),
    TURN(-90,2000),
    BACKWARD(52.5),
    DESCENDRE_BRAS(),
    WAIT(500),
    MONTER_BRAS(),
    FORWARD(10),
    TURN(180,2000),
    FORWARD(15),
    MONTER_ACTIONNEUR(),
    WAIT(500),
    FERMER_AIMANTS(),
    WAIT(500),
    BACKWARD(15),
    DESCENDRE_ACTIONNEUR(),
    OUVRIR_AIMANTS(),
    WAIT(2000),
    FORWARD(15),
    MONTER_ACTIONNEUR(),
    WAIT(2000),
    BACKWARD(62.5,3000),
    TURN(90,1000),
    FORWARD(12.5,3000),
    MONTER_CANNETTE_2E_ETAGE(),
    WAIT(1000),
    FORWARD(13,500),
    MILLIEU_ACTIONNEUR(),
    WAIT(400),
    FERMER_AIMANTS(),
    WAIT(400),
    BACKWARD(10,1000),


};
strategie stratapointyellow = strategie{

};
// coord depart=1800/200
strategie stratadoublepointblue = strategie{
    DESCENDRE_ACTIONNEUR(),
    OUVRIR_AIMANTS(),
    BACKWARD(30),
    TURN(-90),
    FORWARD(42.5),
    TURN(-90),
    BACKWARD(15),
    ACTIVER_POMPE(),
    WAIT(100),
    DESCENDRE_BRAS(),
    WAIT(500),
    MONTER_BRAS(),
    FORWARD(15),
    TURN(180,6000),
    FORWARD(35,4000),
    MONTER_CANNETTE_2E_ETAGE(),
    WAIT(2000),
    FORWARD(10,1000),
    BACKWARD(27.5),
    TURN(180,2000),
    BACKWARD(20),
    DESCENDRE_BRAS(),
    WAIT(500),
    DESACTIVER_POMPE(),
    WAIT(250),
    MONTER_BRAS(),
    FORWARD(20),
    TURN(180,2000),
    FORWARD(24),
    MONTER_ACTIONNEUR(),
    WAIT(500),
    FERMER_AIMANTS(),
    WAIT(500),
    BACKWARD(24.5),
    TURN(-90),
    DESCENDRE_ACTIONNEUR(),
    OUVRIR_AIMANTS(),
    FORWARD(65.5),
    MONTER_ACTIONNEUR(),
    BACKWARD(110),
    TURN(90),
    FORWARD(35),
    FORWARD(5,500),
    BACKWARD(15),
    DESCENDRE_ACTIONNEUR(),
    WAIT(3000),
    FERMER_AIMANTS(),
    WAIT(500),
    BACKWARD(40),
    TURN(-90),
    FORWARD(90),
    TURN(90),
    BACKWARD(80),
    MILIEU_BRAS(),

};
strategie stratadoublepointyellow = strategie{
    DESCENDRE_ACTIONNEUR(),
    OUVRIR_AIMANTS(),
    BACKWARD(30),
    TURN(90),
    FORWARD(42.5),
    TURN(90),
    BACKWARD(15),
    ACTIVER_POMPE(),
    WAIT(100),
    DESCENDRE_BRAS(),
    WAIT(500),
    MONTER_BRAS(),
    FORWARD(15),
    TURN(-180,6000),
    FORWARD(35,4000),
    MONTER_CANNETTE_2E_ETAGE(),
    WAIT(2000),
    FORWARD(10,1000),
    BACKWARD(27.5),
    TURN(-180,2000),
    BACKWARD(20),
    DESCENDRE_BRAS(),
    WAIT(500),
    DESACTIVER_POMPE(),
    MONTER_BRAS(),
    FORWARD(20),
    TURN(180,2000),
    FORWARD(24),
    MONTER_ACTIONNEUR(),
    WAIT(500),*
    FERMER_AIMANTS(),
    WAIT(500),
    BACKWARD(24.5),
    TURN(90),
    DESCENDRE_ACTIONNEUR(),
    OUVRIR_AIMANTS(),
    FORWARD(65.5),
    MONTER_ACTIONNEUR(),
    BACKWARD(110),
    TURN(-90),
    FORWARD(35),
    FORWARD(5,500),
    BACKWARD(15),
    DESCENDRE_ACTIONNEUR(),
    WAIT(3000),
    FERMER_AIMANTS(),
    WAIT(500),
    BACKWARD(40),
    TURN(90),
    FORWARD(90),
    TURN(-90),
    BACKWARD(80),
    MILIEU_BRAS(),
    
};
strategie noforfait2 = strategie{
    OUVRIR_AIMANTS(),
    WAIT(1000),
    FERMER_AIMANTS(),
    MILLIEU_ACTIONNEUR(),
    WAIT(1000),
    DESCENDRE_ACTIONNEUR(),
    BACKWARD(2),
    FORWARD(30 - 13),
    TURN(90),
    FORWARD(10),
    TURN(-90),
    FORWARD(90),
    TURN(90),
    FORWARD(60),
    TURN(-90),
    FORWARD(20),
    TURN(90),
    FORWARD(10),
    TURN(-90),
    FORWARD(60)};
strategie pamisuperstarstratblue = strategie{
    // BACKWARD(2),
    FORWARD(120),
    TURN(90),
    BACKWARD(10),
    FORWARD(38),
    // TURN(45),
    // FORWARD(10,DEFAULT_SPEED/2)
};
strategie pamisuperstarstratyellow = strategie{
    // BACKWARD(2),
    FORWARD(120),
    TURN(-90),
    BACKWARD(10),
    FORWARD(38),
    // TURN(-45),
    // FORWARD(10,DEFAULT_SPEED/2)
};
strategie pamistratyellow = strategie{
    FORWARD(20, DEFAULT_SPEED * 1.5),
    TURN(-45),
    FORWARD(35, DEFAULT_SPEED * 1.5),
    TURN(45),
    FORWARD(40, DEFAULT_SPEED * 1.5),
    WAIT(5000),
    FORWARD(15, DEFAULT_SPEED * 1.5),
    TURN(90),
};
strategie pamistratblue = strategie{
    FORWARD(20, DEFAULT_SPEED * 1.5),
    TURN(45),
    FORWARD(35, DEFAULT_SPEED * 1.5),
    TURN(-45),
    FORWARD(40, DEFAULT_SPEED * 1.5),
    WAIT(5000),
    FORWARD(15, DEFAULT_SPEED * 1.5),
    TURN(-90),
};
/// @brief la stratégie finale du robot (peut être définie sur n'importe quelle stratégie)
strategie strat = stratdemoservo;
void choixStrategie()
{
    // strat = strategie{MILLIEU_ACTIONNEUR(),
    // BACKWARD(25,4000),
    // TURN(-135,4000),
    // FORWARD(sqrt(2) * 10,4000),
    // TURN(-45,4000),
    // FORWARD(30,4000),};
    // return;
    if (pamimode)
    {
        bool isSuperStar = false;
        if (!pcf.digitalRead(0))
        { // YELLOW
            if (isSuperStar)
            {
                strat = pamisuperstarstratyellow;
            }
            else
            {
                strat = pamistratyellow;
            }
        }
        else
        {
            // BLUE
            if (isSuperStar)
            {
                strat = pamisuperstarstratblue;
            }
            else
            {
                strat = pamistratblue;
            }
        }
    }
    else
    {
        if (pcf.digitalRead(1))
        { // YELLOW
            if (pcf.digitalRead(2))
            {
                // STRAT 2 ETAGES
                strat = stratadoublepointblue;
            }
            else
            {
                strat = stratapointblue;
            }
        }
        else
        {
            if (pcf.digitalRead(2))
            {
                // STRAT 2 ETAGES
                strat = stratadoublepointyellow;
            }
            else
            {
                strat = stratapointyellow;
            }
        }
    }
}

// DEBUT DU CODE PUR ET DUR
long matchStartTime = 0;
bool matchStarted = false;
Adafruit_PWMServoDriver pcacard = Adafruit_PWMServoDriver();
#define SERVOMIN 125
#define SERVOMAX 575
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
};

step_state etat_action = step_state::IDLE;

typedef std::vector<etape> strategie;
constexpr int actionneur_ascenseur_offset = 20;
int angleToPulse(int);
void fermer_aimants()
{
    if (pamimode)
        return;
    pcacard.setPWM(Pin::Actuators::Servo::LEFT_MAGNET, 0, angleToPulse(10));
    pcacard.setPWM(Pin::Actuators::Servo::RIGHT_MAGNET, 0, angleToPulse(80));
}
void ouvrir_aimants()
{
    if (pamimode)
        return;
    pcacard.setPWM(Pin::Actuators::Servo::LEFT_MAGNET, 0, angleToPulse(45));
    pcacard.setPWM(Pin::Actuators::Servo::RIGHT_MAGNET, 0, angleToPulse(45));
}
void actionneur_pos(double angle)
{
    if (pamimode)
        return;
    if (angle < 33)
        return;
    // WARNING ANGLE MIN ACTIONNEUR
    // attention minimum à 30 sinon risque de casse
    pcacard.setPWM(Pin::Actuators::Servo::ELEVATOR, 0, angleToPulse(angle));
}
void monter_banderole()
{
    if (pamimode)
        return;
    pcacard.setPWM(Pin::Actuators::Servo::ELEVATOR, 0, angleToPulse(63 + actionneur_ascenseur_offset));
}
void monter_actionneur()
{
    if (pamimode)
        return;
    pcacard.setPWM(Pin::Actuators::Servo::ELEVATOR, 0, angleToPulse(53 + actionneur_ascenseur_offset));
}
void monter_canette_2e_etage()
{
    if (pamimode)
        return;
    pcacard.setPWM(Pin::Actuators::Servo::ELEVATOR, 0, angleToPulse(64 + actionneur_ascenseur_offset));
}
void descendre_actionneur()
{
    if (pamimode)
        return;
    pcacard.setPWM(Pin::Actuators::Servo::ELEVATOR, 0, angleToPulse(13 + actionneur_ascenseur_offset));
}
void millieu_actionneur()
{
    if (pamimode)
        return;
    pcacard.setPWM(Pin::Actuators::Servo::ELEVATOR, 0, angleToPulse(40 + actionneur_ascenseur_offset));
}
void monter_bras()
{
    if (pamimode)
        return;
    pcacard.setPWM(Pin::Actuators::Servo::RIGHT_ARM, 0, angleToPulse(90 -(0-4)));
    pcacard.setPWM(Pin::Actuators::Servo::LEFT_ARM, 0, angleToPulse(0));
}
void milieu_bras()
{
    if (pamimode)
        return;
    pcacard.setPWM(Pin::Actuators::Servo::RIGHT_ARM, 0, angleToPulse(90 - (22-4)));
    pcacard.setPWM(Pin::Actuators::Servo::LEFT_ARM, 0, angleToPulse(22));
}
void descendre_bras()
{
    if (pamimode)
        return;
    pcacard.setPWM(Pin::Actuators::Servo::RIGHT_ARM, 0, angleToPulse(90 - (50-4)));
    pcacard.setPWM(Pin::Actuators::Servo::LEFT_ARM, 0, angleToPulse(50));
}
void activer_pompe()
{
    if (pamimode)
        return;
    pcacard.setPWM(Pin::Actuators::Servo::PUMP_LEFT, 0, 4096);
    pcacard.setPWM(Pin::Actuators::Servo::PUMP_LEFT_VALVE, 4096, 0);
    pcacard.setPWM(Pin::Actuators::Servo::PUMP_RIGHT, 0, 4096);
    pcacard.setPWM(Pin::Actuators::Servo::PUMP_RIGHT_VALVE, 4096, 0);
}
void desactiver_pompe()
{
    if (pamimode)
        return;
    pcacard.setPWM(Pin::Actuators::Servo::PUMP_LEFT, 4096, 0);
    pcacard.setPWM(Pin::Actuators::Servo::PUMP_LEFT_VALVE, 0, 4096);
    pcacard.setPWM(Pin::Actuators::Servo::PUMP_RIGHT, 4096, 0);
    pcacard.setPWM(Pin::Actuators::Servo::PUMP_RIGHT_VALVE, 0, 4096);
}

int angleToPulse(int angle)
{
    int pulse = map(angle, 0, 90, SERVOMIN, SERVOMAX);
    return pulse;
}
void wait_end()
{
    while (matchStarted && millis() - matchStartTime <= 94000)
        ;
}

void debugMode();

int etapeencour = -1;
/// @brief Appeler la fonction correspondant à une étape
/// @param step l'étape actuelle
void actioncall(etape step)
{
    switch (step.action)
    {
    case atype::FORWARD:

        return robot.forward(step.distance, step.vitesse);
        break;

    case atype::BACKWARD:
        // sendCurrentAngle({0, -1});
        return robot.backward(step.distance, step.vitesse);
        break;

    case atype::TURN:
        // if (step.angle > 0)
        // {
        //     sendCurrentAngle({0.5, 0.5});
        // }
        // else if (step.angle < 0)
        // {
        //     sendCurrentAngle({-0.5, 0.5});
        // }
        // else
        // {
        //     sendCurrentAngle({0, 1});
        // }
        return robot.turn(step.angle, step.vitesse);

    case atype::TURNTO:
        return robot.turnTo(step.angle, step.vitesse);
        break;

    case atype::MOVETO:
        return robot.moveTo(step.coordonnees, step.vitesse);
    case atype::OUVRIR_AIMANTS:
        return ouvrir_aimants();
        break;
    case atype::FERMER_AIMANTS:
        return fermer_aimants();
        break;
    case atype::MONTER_ACTIONNEUR:
        return monter_actionneur();
        break;
    case atype::DESCENDRE_ACTIONNEUR:
        return descendre_actionneur();
        break;
    case atype::MILLIEU_ACTIONNEUR:
        return millieu_actionneur();
        break;
    case atype::MONTER_BRAS:
        return monter_bras();
        break;
    case atype::DESCENDRE_BRAS:
        return descendre_bras();
        break;
    case atype::ACTIVER_POMPE:
        return activer_pompe();
        break;
    case atype::DESACTIVER_POMPE:
        return desactiver_pompe();
        break;
    case atype::WAIT:
        return delay(step.time);
        break;
    case atype::MONTER_BANDEROLE:
        return monter_banderole();
        break;
    case atype::MONTER_CANETTE_2E_ETAGE:
        return monter_canette_2e_etage();
        break;
    case atype::WAIT_END:
        return wait_end();
        break;
    case atype::ACTIONNEUR_POS:
        return actionneur_pos(step.angle);
    default:
        break;
    }
}

/// @brief vérifier si une étape est finie
/// @return vrai si l'étape est finie, sinon faux
bool actionfini()
{
    return robot.reachedtarget();
}

int etape_initbanderole = -1;
void initialisation_et_banderole()
{
    if (pamimode)
        return;
    double distance_recul_banderole = 3.5;
    strategie init_and_banderole = strategie{
        MONTER_BANDEROLE(),
        WAIT(2000),
        FORWARD(22,1000),
        BACKWARD(distance_recul_banderole),
        MILLIEU_ACTIONNEUR(),
        WAIT(1000),
        BACKWARD(20 - distance_recul_banderole),
        // FERMER_AIMANTS(),
    };
    step_state init_sub_state = step_state::IDLE;
    while (1)
    {
        if (init_sub_state == step_state::RUNNING and actionfini())
        {
            init_sub_state = step_state::IDLE;
            debugPrintln("end");
            // debugPrintln("running one !");
        }
        else if (init_sub_state == step_state::RUNNING)
        {
            bool *isStopped;
            bool pppp = false;
            isStopped = &pppp;
            robot.run(isStopped);
        }
        else if (init_sub_state == step_state::IDLE)
        {
            if (etape_initbanderole + 1 == init_and_banderole.size())
            {
                etape_initbanderole++;
                return;
            }
            if (etape_initbanderole + 1 > init_and_banderole.size())
            {
                return;
            }
            etape_initbanderole++;
            actioncall(init_and_banderole[etape_initbanderole]);
            init_sub_state = step_state::RUNNING;
        }
        else
        {
            init_sub_state = step_state::IDLE;
        }
    }
}

int etape_inittable = -1;
void initialisation_table()
{
    if (pamimode)
        return;
    bool isYellow = !pcf.digitalRead(1);
    strategie init_table = strategie{
        MONTER_BRAS(),
        DESACTIVER_POMPE(),
        MILLIEU_ACTIONNEUR(),
        BACKWARD(5, DEFAULT_SPEED / 2),
        FORWARD(122.5 - 13),
        TURN(isYellow ? 90 : -90),
        FORWARD(15, 1000 ),
        BACKWARD(10),
        // DESCENDRE_BRAS(),
        // ACTIVER_POMPE()
    };
    step_state init_sub_state = step_state::IDLE;
    while (1)
    {
        if (init_sub_state == step_state::RUNNING and actionfini())
        {
            init_sub_state = step_state::IDLE;
            debugPrintln("end");
            // debugPrintln("running one !");
        }
        else if (init_sub_state == step_state::RUNNING)
        {
            bool *isStopped;
            bool pppp = false;
            isStopped = &pppp;
            robot.run(isStopped);
        }
        else if (init_sub_state == step_state::IDLE)
        {
            if (etape_inittable + 1 == init_table.size())
            {
                etape_inittable++;
                return;
            }
            if (etape_inittable + 1 > init_table.size())
            {
                return;
            }
            etape_inittable++;
            actioncall(init_table[etape_inittable]);
            init_sub_state = step_state::RUNNING;
        }
        else
        {
            init_sub_state = step_state::IDLE;
        }
    }
}

bool debugmode = false;

// pour redémarrer le code depuis bouton ou autre : ESP.restart() ou abort()

bool activer_lidar = true;
/// @brief fonction d'initialisation
void setup()
{
    Serial.begin(9600);
    pcacard.begin();
    pcacard.setPWMFreq(60);
    pinMode(D8, OUTPUT);
    digitalWrite(D8, LOW);
    // if(false){//TEST
    delay(1000);
    activer_pompe();
    // delay(200);
    // debugPrintln("monter_bras");
    // monter_bras();
    // delay(3000);
    // debugPrintln("milieu_bras");
    // milieu_bras();
    // activer_pompe();
    // delay(3000);
    // debugPrintln("descendre_bras");
    // descendre_bras();
    // delay(3000);
    // milieu_bras();
    // delay(5000);
    // descendre_bras();
    // desactiver_pompe();
    // }
    if (!pcf.begin(0x20, &Wire))
    {
        Serial.println("Couldn't find PCF8574");
        while (1)
        {
            debugPrintln("Failled to find PCF");
            delay(1000);
        };
    }
    // pcf.begin();
    pcf.pinMode(0, INPUT);
    pcf.pinMode(1, INPUT);
    pcf.pinMode(2, INPUT);
    pcf.pinMode(3, INPUT);
    pcf.pinMode(4, OUTPUT);
    pinMode(Pin::IHM::TIRETTE, INPUT_PULLUP);
    delay(2000);
    if (!pamimode)
    {
        // delay(2000);
        // // descendre_actionneur();
        // while(1);
        // callibrage bordure
        debugPrintln("ready to check");
        while (pcf.digitalRead(0))
            ; //{debugPrintln("checking");delay(1000);};
        delay(2000);
        initialisation_table();
        delay(1000);
    }
    while (!digitalRead(Pin::IHM::TIRETTE))
    {
        debugPrintln("what the fuck?");
        delay(100);
    }
    // fonction de pre_init
    while (digitalRead(Pin::IHM::TIRETTE))
    {
        // debugPrintln("Mais t'es pas là mais t'es où?");
        // delay(100);
    }
    if (!pcf.digitalRead(1) && !pcf.digitalRead(2) && !pcf.digitalRead(3))
    {
        // enter debug mode
        debugmode = true;
    }
    if (!pcf.digitalRead(3))
    {
        activer_lidar = false;
    }
    choixStrategie();
    millieu_actionneur();
    ouvrir_aimants();
    // delay(3000);
    // monter_banderole();
    etat_a = etat::INITALISATION;
    delay(1000);
}

bool showed_step = false;
bool isPaused = false;
long lastPamiDetectCheck = 0;
bool lastPamiDetectValue = false;
double angle = 0;
/// @brief fonction appelée à chaque loop du controlleur
void loop()
{
    // if(debugmode){
    //     debugMode();
    //     return;
    // }
    if (etat_a != etat::MATCH)
    {
        if (etat_a == etat::FIN)
        {
            if (!pamimode)
                return;
            angle += .1;
            pcacard.setPWM(0, 0, 45 + (cos(angle) * 45));
            pcacard.setPWM(15, 0, 45 + (cos(angle) * 45));
            debugPrintln(45 + (cos(angle) * 45));
            delay(1000);
            return;
        }
        if (not(digitalRead(Pin::IHM::TIRETTE)))
        {
            // debugPrintln("tirette présente");
        }
        else
        {
            etat_a = etat::MATCH;
            // debugPrintln("fin setup");
            matchStarted = true;
            matchStartTime = millis();
            initialisation_et_banderole();
            if (pamimode)
            {
                delay(PAMI_WAIT);
            }
            initLidar();
        }
        // delay(500);
        return;
    }
    if (matchStarted && millis() - matchStartTime >= 100000)
    {
        etat_a = etat::FIN;
        return;
    }
    if (etat_action == step_state::RUNNING and actionfini())
    {
        etat_action = step_state::IDLE;
    }
    else if (etat_action == step_state::RUNNING)
    {
        bool *isStopped;
        // bool temppause = false;
        // isStopped=&temppause;
        if (activer_lidar)
        {
            isStopped = getLidarStatus();
        }
        else
        {
            bool hello = false;
            isStopped = &hello;
        }
        robot.run(isStopped);
    }
    else if (etat_action == step_state::IDLE)
    {
        if (etapeencour + 1 == strat.size())
        { // etapesuivante =etapeencours + 1; par rapport à size() = etapesuivante -1; donc = etapeencour+1-1
            // debugPrintln("hello fucking world ?");
            // debugPrintln(etapeencour);
            // debugPrintln(strat.size());
            // debugPrintln("ok");
            // delay(250);
            etat_a = etat::FIN;
            etapeencour++;
            etat_a = etat::FIN;
            return;
        }
        if (etapeencour + 1 > strat.size())
        {
            etat_a = etat::FIN;
            return;
        }
        // showed_step = false;
        etapeencour++;
        debugPrint("calling action backward with speed");
        debugPrint(strat[etapeencour].vitesse);
        actioncall(strat[etapeencour]);
        etat_action = step_state::RUNNING;
        // debugPrint("running three ! at step");
        // debugPrintln(etapeencour);
        // delay(100);
    }
    else
    {
        etat_action = step_state::IDLE;
        // debugPrintln("running four !");
        // delay(200);
    }
}

void debugMode()
{
    // if(!pcf.digitalRead(0)){
    //     debugPrint("helloP0");debugPrintln(millis());
    // }
    // if(!pcf.digitalRead(1)){
    //     debugPrint("helloP1");debugPrintln(millis());
    // }
    // if(!pcf.digitalRead(2)){
    //     debugPrint("helloP2");debugPrintln(millis());
    // }
    // if(!pcf.digitalRead(3)){
    //     debugPrint("helloP3");debugPrintln(millis());
    // }
    // delay(500);
    int code = 0;
    if (!pcf.digitalRead(3))
        code += 1;
    if (!pcf.digitalRead(2))
        code += 10;
    if (!pcf.digitalRead(1))
        code += 100;
    debugPrintln(((std::string) "debugMode: code actuel en écriture: " + std::to_string(code)).c_str());
    if (!pcf.digitalRead(0))
    {
        // CODE 100 = MONTER
        if (code == 100)
        {
            monter_actionneur();
            debugPrintln("monter actionneur");
        }
        // CODE 001 = DESCENDRE
        if (code == 1)
        {
            descendre_actionneur();
            debugPrintln("descendre actionneur");
        }
        // CODE 110 = ouvrir aimants
        if (code == 110)
        {
            ouvrir_aimants();
            debugPrintln("ouvrir aimants");
        }
        // CODE 011 = fermer aimants
        if (code == 11)
        {
            fermer_aimants();
            debugPrintln("fermer aimants");
        }
        // CODE 010 = milieu actionneur
        if (code == 10)
        {
            millieu_actionneur();
            debugPrintln("milieu actionneur");
        }
        debugPrintln("activation code");
    }
    // delay(250);
}
