// TO SET STRATEGIE SEARCH STRATDEF
#include <Arduino.h>
#include <lidar.h>
#include <Robotmove.h>
#include <Adafruit_PCF8574.h>
#include <vector>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <constante.h>
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
    OUVRIR_BRAS,
    FERMER_BRAS,
    ACTIVER_POMPE,
    DESACTIVER_POMPE,
    WAIT,
    MONTER_BANDEROLE
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
        1000
#else
        8000
#endif
    ;
etape FORWARD(double d, int v = DEFAULT_SPEED) { return etape{.action = A::FORWARD, .distance = d, .vitesse = v}; }
etape BACKWARD(double d, int v = DEFAULT_SPEED) { return etape{.action = A::BACKWARD, .distance = d, .vitesse = v}; }
etape TURN(double a, int v = DEFAULT_SPEED) { return etape{.action = A::TURN, .angle = a, .vitesse = v}; }
etape TURNTO(double a, int v = DEFAULT_SPEED) { return etape{.action = A::TURNTO, .angle = a, .vitesse = v}; }
etape MOVETO(RobotMove::Coord c, int v = DEFAULT_SPEED) { return etape{.action = A::MOVETO, .coordonnees = c, .vitesse = v}; }
etape FERMER_AIMANTS() { return etape{.action = A::FERMER_AIMANTS}; }
etape OUVRIR_AIMANTS() { return etape{.action = A::OUVRIR_AIMANTS}; }
etape MONTER_ACTIONNEUR() { return etape{.action = A::MONTER_ACTIONNEUR}; }
etape DESCENDRE_ACTIONNEUR() { return etape{.action = A::DESCENDRE_ACTIONNEUR}; }
etape MILLIEU_ACTIONNEUR() { return etape{.action = A::MILLIEU_ACTIONNEUR}; }
etape OUVRIR_BRAS() { return etape{.action = A::OUVRIR_BRAS}; }
etape FERMER_BRAS() { return etape{.action = A::FERMER_BRAS}; }
etape ACTIVER_POMPE() { return etape{.action = A::ACTIVER_POMPE}; }
etape DESACTIVER_POMPE() { return etape{.action = A::DESACTIVER_POMPE}; }
etape MONTER_BANDEROLE() { return etape{.action = A::MONTER_BANDEROLE}; }
etape WAIT(int time) { return etape{.action = A::WAIT, .time = time}; }
typedef std::vector<etape> strategie;
Adafruit_PCF8574 pcf;


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
    // OUVRIR_BRAS(),
    // WAIT(2000),
    // FERMER_BRAS(),
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
    BACKWARD(40)};
strategie stratapointblue = strategie{
    FORWARD(40 - 13),
    TURN(90),
    FORWARD(12.5),
    TURN(-90),
    FORWARD(40),
    DESCENDRE_ACTIONNEUR(),
    OUVRIR_AIMANTS(),
    WAIT(1000),
    FORWARD(20),
    MILLIEU_ACTIONNEUR(),
    BACKWARD(20),
    TURN(-90),
    FORWARD(12.5),
    TURN(-90),
    FORWARD(60),
    DESCENDRE_ACTIONNEUR(),
    FERMER_AIMANTS(),
    BACKWARD(10),
    TURN(-90),
    FORWARD(12.5),
    TURN(-90),
    FORWARD(90),
    TURN(90),
    FORWARD(60),
    TURN(-90),
    FORWARD(20),
    TURN(90),
    FORWARD(12.5),
    TURN(-90),
    FORWARD(60)

};
strategie stratapointyellow = strategie{
    BACKWARD(2),
    FORWARD(40 - 13),
    TURN(-90),
    FORWARD(12.5),
    TURN(90),
    FORWARD(40),
    DESCENDRE_ACTIONNEUR(),
    OUVRIR_AIMANTS(),
    WAIT(1000),
    FORWARD(20),
    MILLIEU_ACTIONNEUR(),
    BACKWARD(20),
    TURN(90),
    FORWARD(12.5),
    TURN(90),
    FORWARD(60),
    DESCENDRE_ACTIONNEUR(),
    FERMER_AIMANTS(),
    BACKWARD(10),
    TURN(90),
    FORWARD(12.5),
    TURN(90),
    FORWARD(90),
    TURN(-90),
    FORWARD(60),
    TURN(90),
    FORWARD(20),
    TURN(-90),
    FORWARD(12.5),
    TURN(90),
    FORWARD(60)

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
strategie pamistratblue = strategie{
    // BACKWARD(2),
    FORWARD(120),
    TURN(-90),
    FORWARD(20),
    TURN(45),
    FORWARD(10,500)
};
strategie pamistratyellow = strategie{
    // BACKWARD(2),
    FORWARD(120),
    TURN(90),
    FORWARD(20)};
/// @brief la stratégie finale du robot (peut être définie sur n'importe quelle stratégie)
strategie strat = stratdemoservo;
void choixStrategie()
{
    // while (1)
    // {
    // // en théorie code 111 réservé pour debugMode donc préferable de ne pas utiliser
    // int code = 0;

    // if (!pcf.digitalRead(3))
    //     code += 1;
    // if (!pcf.digitalRead(2))
    //     code += 10;
    // if (!pcf.digitalRead(1))
    //     code += 100;
    // if(!pcf.digitalRead(0))code+=1000;
    // debugPrintln(((std::string) "debugMode: code actuel en écriture: " + std::to_string(code)).c_str());
    // if (code == 1)
    // {
    // }
    // else if (code == 10)
    // {
    // }
    // delay(1000);
    // }
    if (pamimode)
    {
        if (!pcf.digitalRead(1))
        { // YELLOW
            strat = pamistratyellow;
        }
        else
        {
            // BLUE
            strat = pamistratblue;
        }
    }
    else
    {
        if (!pcf.digitalRead(1))
        { // YELLOW
            strat = stratapointblue;
        }
        else
        {
            strat = stratapointyellow;
        }
    }
}

// DEBUT DU CODE PUR ET DUR

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

int angleToPulse(int);
void fermer_aimants()
{
    if (pamimode)
        return;
    pcacard.setPWM(13, 0, angleToPulse(20));
    pcacard.setPWM(14, 0, angleToPulse(70));
    delay(1000);
}
void ouvrir_aimants()
{
    if (pamimode)
        return;
    pcacard.setPWM(13, 0, angleToPulse(45));
    pcacard.setPWM(14, 0, angleToPulse(45));
    delay(1000);
}
void monter_banderole()
{
    if (pamimode)
        return;
    pcacard.setPWM(0, 0, angleToPulse(60));
}
void monter_actionneur()
{
    if (pamimode)
        return;
    pcacard.setPWM(0, 0, angleToPulse(50));
    delay(1000);
}
void descendre_actionneur()
{
    if (pamimode)
        return;
    pcacard.setPWM(0, 0, angleToPulse(15));
    delay(3000);
}
void millieu_actionneur()
{
    if (pamimode)
        return;
    pcacard.setPWM(0, 0, angleToPulse(37));
    delay(1000);
}
void ouvrir_bras()
{
    if (pamimode)
        return;
    pcacard.setPWM(11, 0, angleToPulse(90 - 0));
    pcacard.setPWM(12, 0, angleToPulse(0));
    delay(1000);
}
void fermer_bras()
{
    if (pamimode)
        return;
    pcacard.setPWM(11, 0, angleToPulse(90 - 45));
    pcacard.setPWM(12, 0, angleToPulse(45));
    delay(1000);
}
void activer_pompe()
{
    if (pamimode)
        return;
    pcf.digitalWrite(5, HIGH);
}
void desactiver_pompe()
{
    if (pamimode)
        return;
    pcf.digitalWrite(5, LOW);
}
int angleToPulse(int angle)
{
    int pulse = map(angle, 0, 90, SERVOMIN, SERVOMAX);
    return pulse;
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
        sendCurrentAngle({0, -1});
        return robot.backward(step.distance, step.vitesse);
        break;

    case atype::TURN:
        if (step.angle > 0)
        {
            sendCurrentAngle({0.5, 0.5});
        }
        else if (step.angle < 0)
        {
            sendCurrentAngle({-0.5, 0.5});
        }
        else
        {
            sendCurrentAngle({0, 1});
        }
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
    case atype::OUVRIR_BRAS:
        return ouvrir_bras();
        break;
    case atype::FERMER_BRAS:
        return fermer_bras();
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

void initialisation_et_banderole()
{
    if(pamimode)return;
    strategie init_and_banderole = strategie{
        MONTER_BANDEROLE(),
        WAIT(2000),
        FORWARD(20),
        BACKWARD(3.5),
        MONTER_ACTIONNEUR(),
        BACKWARD(20 - 3.5),
        FERMER_AIMANTS(),
        TURN(180),
        BACKWARD(10),
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
            if (etapeencour + 1 == init_and_banderole.size())
            {
                etapeencour++;
                return;
            }
            if (etapeencour + 1 > init_and_banderole.size())
            {
                return;
            }
            etapeencour++;
            actioncall(init_and_banderole[etapeencour]);
            init_sub_state = step_state::RUNNING;
            debugPrint("running three ! at step");
            debugPrintln(etapeencour);
            delay(100);
        }
        else
        {
            init_sub_state = step_state::IDLE;
            debugPrintln("running four !");
            delay(200);
        }
    }
}


void initialisation_table()
{
    if(pamimode)return;
    strategie init_table = strategie{
        BACKWARD(5),
        FORWARD(30-13),
        TURN(-90),
        BACKWARD(122),
        FORWARD(120+13),
        TURN(-90),
        FORWARD(20)
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
            if (etapeencour + 1 == init_table.size())
            {
                etapeencour++;
                return;
            }
            if (etapeencour + 1 > init_table.size())
            {
                return;
            }
            etapeencour++;
            actioncall(init_table[etapeencour]);
            init_sub_state = step_state::RUNNING;
            debugPrint("running three ! at step");
            debugPrintln(etapeencour);
            delay(100);
        }
        else
        {
            init_sub_state = step_state::IDLE;
            debugPrintln("running four !");
            delay(200);
        }
    }
}

bool debugmode = false;

/// @brief fonction d'initialisation
void setup()
{
    Serial.begin(9600);
    pcacard.begin();
    pcacard.setPWMFreq(60);
     if (!pcf.begin(0x20, &Wire)) {
    Serial.println("Couldn't find PCF8574");
    while (1);
  }
    // pcf.begin();
    pcf.pinMode(0, INPUT);
    pcf.pinMode(1, INPUT);
    pcf.pinMode(2, INPUT);
    pcf.pinMode(3, INPUT);
    pcf.pinMode(4, OUTPUT);
    pinMode(D8, OUTPUT);
    digitalWrite(D8, LOW);
    delay(1000);
    pinMode(Pin::IHM::TIRETTE, INPUT_PULLUP);
    while (!digitalRead(Pin::IHM::TIRETTE))
    {
        debugPrintln("what the fuck?");
        delay(100);
    }
    // fonction de pre_init
    while (digitalRead(Pin::IHM::TIRETTE))
    {
        // debugPrintln("Mais t'es pas là mais t'es où?");
        delay(100);
    }
    if (!pcf.digitalRead(1) && !pcf.digitalRead(2) && !pcf.digitalRead(3))
    {
        // enter debug mode
        debugmode = true;
    }
    choixStrategie();
    millieu_actionneur();
    ouvrir_aimants();
    initialisation_table();
    // delay(3000);
    // monter_banderole();
    etat_a = etat::INITALISATION;
    delay(1000);
}

bool showed_step = false;
bool isPaused = false;
long lastPamiDetectCheck = 0;
bool lastPamiDetectValue = false;
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
            return;
        }
        if (not(digitalRead(Pin::IHM::TIRETTE)))
        {

            debugPrintln("Jchuis là");
            // debugPrintln(hc.dist());
            // delay(100);
        }
        else
        {
            etat_a = etat::MATCH;
            debugPrintln("fin setup");
            initialisation_et_banderole();
            initLidar();
            delay(5000);
            // delay(100);
        }
        delay(100);
        return;
    }
    // if (getLidarStatus())
    // {
    //     debugPrint("paused by lidar at ");
    //     // delay(200);
    //     if (!isPaused)
    //     {
    //         robot.stop();
    //         isPaused = true;
    //     }
    //     robot.run(getLidarStatus());
    //     return;
    // }
    // else
    // {
    //     if (isPaused)
    //     {
    //         robot.resume();
    //         isPaused = false;
    //     }
    // }
    if (etat_action == step_state::RUNNING and actionfini())
    {
        etat_action = step_state::IDLE;
        debugPrintln("end");
        // debugPrintln("running one !");
    }
    else if (etat_action == step_state::RUNNING)
    {
        if (!showed_step)
        {
            debugPrintln("hello");
            delay(100);
            showed_step = true;
        }
        bool *isStopped;
        isStopped = getLidarStatus();
        robot.run(isStopped);
        // robot.debugPosition();
        // debugPrintln("running two !");
        // delay(100);
    }
    else if (etat_action == step_state::IDLE)
    {
        if (etapeencour + 1 == strat.size())
        { // etapesuivante =etapeencours + 1; par rapport à size() = etapesuivante -1; donc = etapeencour+1-1
            debugPrintln("hello fucking world ?");
            debugPrintln(etapeencour);
            debugPrintln(strat.size());
            debugPrintln("ok");
            delay(250);
            etapeencour++;
            return;
        }
        if (etapeencour + 1 > strat.size())
        {
            return;
        }
        showed_step = false;
        etapeencour++;
        actioncall(strat[etapeencour]);
        etat_action = step_state::RUNNING;
        debugPrint("running three ! at step");
        debugPrintln(etapeencour);
        delay(100);
    }
    else
    {
        etat_action = step_state::IDLE;
        debugPrintln("running four !");
        delay(200);
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