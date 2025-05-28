// TO SET STRATEGIE SEARCH STRATDEF
#include <Arduino.h>
#include <lidar.h>
#include <Robotmove.h>
#include <PCF8574.h>
#include <vector>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// CODE NECESSAIRE AVANT DECLARATION DE STRATEGIE
enum struct atype{FORWARD,BACKWARD,TURN,TURNTO,MOVETO,FERMER_AIMANTS,OUVRIR_AIMANTS,MONTER_ACTIONNEUR,DESCENDRE_ACTIONNEUR,MILLIEU_ACTIONNEUR,OUVRIR_BRAS,FERMER_BRAS,ACTIVER_POMPE,DESACTIVER_POMPE,WAIT};
typedef atype A;
struct etape{atype action;int distance;RobotMove::Coord coordonnees;double angle;int vitesse;int time;};
constexpr int DEFAULT_SPEED=8000;
etape FORWARD(int d,int v=DEFAULT_SPEED){return etape{.action=A::FORWARD,.distance=d,.vitesse=v};}
etape BACKWARD(int d,int v=DEFAULT_SPEED){return etape{.action=A::BACKWARD,.distance=d,.vitesse=v};}
etape TURN(double a,int v=DEFAULT_SPEED){return etape{.action=A::TURN, .angle=a,.vitesse=v};}
etape TURNTO(double a,int v=DEFAULT_SPEED){return etape{.action=A::TURNTO, .angle=a,.vitesse=v};}
etape MOVETO(RobotMove::Coord c,int v=DEFAULT_SPEED){return etape{.action=A::MOVETO, .coordonnees=c,.vitesse=v};}
etape FERMER_AIMANTS(){return etape{.action=A::FERMER_AIMANTS};}
etape OUVRIR_AIMANTS(){return etape{.action=A::OUVRIR_AIMANTS};}
etape MONTER_ACTIONNEUR(){return etape{.action=A::MONTER_ACTIONNEUR};}
etape DESCENDRE_ACTIONNEUR(){return etape{.action=A::DESCENDRE_ACTIONNEUR};}
etape MILLIEU_ACTIONNEUR(){return etape{.action=A::MILLIEU_ACTIONNEUR};}
etape OUVRIR_BRAS(){return etape{.action=A::OUVRIR_BRAS};}
etape FERMER_BRAS(){return etape{.action=A::FERMER_BRAS};}
etape ACTIVER_POMPE(){return etape{.action=A::ACTIVER_POMPE};}
etape DESACTIVER_POMPE(){return etape{.action=A::DESACTIVER_POMPE};}
etape WAIT(int time){return etape{.action=A::WAIT,.time=time};}
typedef std::vector<etape> strategie;

// DÉFINITION DE LA STRATÉGIE

/// @brief La stratégie numéro un du robot
strategie stratdemo = strategie{
    MILLIEU_ACTIONNEUR(),
    WAIT(2000),
    DESCENDRE_ACTIONNEUR(),
    WAIT(1000),
    OUVRIR_AIMANTS(),
    WAIT(2000),
    FERMER_AIMANTS(),
    WAIT(1000),
    ACTIVER_POMPE(),
    WAIT(2000),
    DESACTIVER_POMPE(),
};

/// @brief la stratégie finale du robot (peut être définie sur n'importe quelle stratégie)
strategie strat = stratdemo;

// DEBUT DU CODE PUR ET DUR

PCF8574 pcf(0x20);
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
    pcacard.setPWM(13,0,angleToPulse(70));
    pcacard.setPWM(14,0,angleToPulse(70));
    delay(1000);
}
void ouvrir_aimants(){
    pcacard.setPWM(13,0,angleToPulse(45));
    pcacard.setPWM(14,0,angleToPulse(45));
    delay(1000);
}
void monter_actionneur(){
    pcacard.setPWM(15,0,angleToPulse(50));
    delay(1000);
}
void descendre_actionneur(){
    pcacard.setPWM(15,0,angleToPulse(23));
    delay(3000);
}
void millieu_actionneur(){
    pcacard.setPWM(15,0,angleToPulse(45));
    delay(1000);
}
void ouvrir_bras(){
    pcacard.setPWM(11,0,angleToPulse(90-0));
    pcacard.setPWM(12,0,angleToPulse(0));
    delay(1000);
}
void fermer_bras(){
    pcacard.setPWM(11,0,angleToPulse(90-45));
    pcacard.setPWM(12,0,angleToPulse(45));
    delay(1000);
}
void activer_pompe(){
    pcf.digitalWrite(P5,HIGH);
}
void desactiver_pompe(){
    pcf.digitalWrite(P5,LOW);
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
        sendCurrentAngle({0,-1});
        return robot.backward(step.distance, step.vitesse);
        break;

    case atype::TURN:
        if(step.angle>0){
            sendCurrentAngle({0.5,0.5});
        }else if(step.angle<0){
            sendCurrentAngle({-0.5,0.5});
        }else{
            sendCurrentAngle({0,1});
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

bool debugmode=false;

/// @brief fonction d'initialisation
void setup()
{
    Serial.begin(115200);
    pcacard.begin();
    pcacard.setPWMFreq(60);
    pcf.begin();
    pcf.pinMode(P0,INPUT);
    pcf.pinMode(P1,INPUT);
    pcf.pinMode(P2,INPUT);
    pcf.pinMode(P3,INPUT);
    pcf.pinMode(P4,OUTPUT);
    delay(1000);
    pinMode(Pin::IHM::TIRETTE, INPUT_PULLUP);
    initLidar();
    while (!digitalRead(Pin::IHM::TIRETTE))
    {
        Serial.println("what the fuck?");
        delay(100);
    }
    // fonction de pre_init
    while (digitalRead(Pin::IHM::TIRETTE))
    {
        Serial.println("Mais t'es pas là mais t'es où?");
        delay(100);
    }
    if(!pcf.digitalRead(P1)&&!pcf.digitalRead(P2)&&!pcf.digitalRead(P3)){
        //enter debug mode
        debugmode=true;
    }
    pinMode(D8, OUTPUT);
    digitalWrite(D8, LOW);
    etat_a = etat::INITALISATION;
    delay(1000);
}

bool showed_step = false;
bool isPaused=false;
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
    if(getLidarStatus()){
        // debugPrint("paused by lidar at ");debugPrintln(millis());
        // delay(200);
        if(!isPaused){robot.stop();isPaused=true;}
        robot.run();
        return;
    }
    else{
        if(isPaused){robot.resume();isPaused=false;}
    }
    if (etat_action == step_state::RUNNING and actionfini())
    {
        etat_action = step_state::IDLE;
        Serial.println("end");
        // Serial.println("running one !");
    }
    else if (etat_action == step_state::RUNNING)
    {
        if (!showed_step)
        {
            Serial.println("hello");
            delay(100);
            showed_step = true;
        }
        robot.run();
        // robot.debugPosition();
        // Serial.println("running two !");
        // delay(100);
    }
    else if (etat_action == step_state::IDLE)
    {
        if (etapeencour+1== strat.size())
        { // etapesuivante =etapeencours + 1; par rapport à size() = etapesuivante -1; donc = etapeencour+1-1
            debugPrintln("hello fucking world ?");
            debugPrintln(etapeencour);
            debugPrintln(strat.size());
            debugPrintln("ok");
            delay(250);
            etapeencour++;
            return;
        }
        if(etapeencour+1>strat.size()){
            return;
        }
        showed_step = false;
        etapeencour++;
        actioncall(strat[etapeencour]);
        etat_action = step_state::RUNNING;
        Serial.print("running three ! at step");
        Serial.println(etapeencour);
        delay(100);
    }
    else
    {
        etat_action = step_state::IDLE;
        Serial.println("running four !");
        delay(200);
    }
}

void debugMode(){
    // if(!pcf.digitalRead(P0)){
    //     debugPrint("helloP0");debugPrintln(millis());
    // }
    // if(!pcf.digitalRead(P1)){
    //     debugPrint("helloP1");debugPrintln(millis());
    // }
    // if(!pcf.digitalRead(P2)){
    //     debugPrint("helloP2");debugPrintln(millis());
    // }
    // if(!pcf.digitalRead(P3)){
    //     debugPrint("helloP3");debugPrintln(millis());
    // }
    // delay(500);
    int code = 0;
    if(!pcf.digitalRead(P3)) code+=1;
    if(!pcf.digitalRead(P2)) code+=10;
    if(!pcf.digitalRead(P1)) code+=100;
    debugPrintln(((std::string)"debugMode: code actuel en écriture: "+std::to_string(code)).c_str());
    if(!pcf.digitalRead(P0)){
        // CODE 100 = MONTER
        if(code==100){
            monter_actionneur();
            debugPrintln("monter actionneur");
        }
        // CODE 001 = DESCENDRE
        if(code==1){
            descendre_actionneur();
            debugPrintln("descendre actionneur");
        }
        // CODE 110 = ouvrir aimants
        if(code==110){
            ouvrir_aimants();
            debugPrintln("ouvrir aimants");
        }
        // CODE 011 = fermer aimants
        if(code==11){
            fermer_aimants();
            debugPrintln("fermer aimants");
        }
        // CODE 010 = milieu actionneur
        if(code==10){
            millieu_actionneur();
            debugPrintln("milieu actionneur");
        }
        debugPrintln("activation code");
    }
    delay(250);
}