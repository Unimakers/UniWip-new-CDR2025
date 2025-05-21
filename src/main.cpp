#include <Arduino.h>
#define HELLOWORLD 3
#if HELLOWORLD == 0
#include <AccelStepper.h>
AccelStepper left, right;
void setup()
{
    pinMode(D8, OUTPUT);
    pinMode(D10, INPUT_PULLUP);
    Serial.begin(115200);
    delay(3000);
    Serial.println("helloworld how are you??");
    while (!digitalRead(D10))
    {
        Serial.println("what the fuck?");
        delay(100);
    }
    // fonction de pre_init
    while (digitalRead(D10))
    {
        Serial.println("Mais t'es pas là mais t'es où?");
        delay(100);
    }
    while (!digitalRead(D10))
    {
        Serial.println("ok t la");
        delay(100);
    }
    right = AccelStepper(AccelStepper::DRIVER, D3, D2);
    left = AccelStepper(AccelStepper::DRIVER, D1, D0);
    left.setAcceleration(500);
    left.setMaxSpeed(1000);
    right.setMaxSpeed(1000);
    right.setAcceleration(500);

    digitalWrite(D8, LOW);
    delay(1000);
    left.move(100000);
    right.move(100000);
    // delay(2000);
    Serial.println("Start");
}
void loop()
{
    left.run();
    right.run();
    // Serial.println("helloow");
    // delay(100);
}
#elif HELLOWORLD == 1
#include <RPLidar.h>
#include <string>
#include <SPI.h>
#include <Wire.h>
#include <constante.h>
RPLidar lidar;
HardwareSerial lidarSerial(1);
// TaskHandle_t Task0;
// void LidarProcess(void *);
void setup()
{
    Serial.begin(115200);
    debugPrintln("setup");
    pinMode(Pin::IHM::LIDAR_PWM, OUTPUT);
    analogWrite(Pin::IHM::LIDAR_PWM, 150);
    // xTaskCreatePinnedToCore(LidarProcess, "Task0", 1000, NULL, 1, &Task0, 0);
    // lidarSerial.end();
    // lidarSerial.begin(115200, SERIAL_8N1,TX,RX); //attention le code ici est changé
    lidar.begin(lidarSerial, Pin::IHM::LIDAR_RX, Pin::IHM::LIDAR_TX);
    lidar.startScan();
}
typedef struct RPLidarMeasurement RPLIDARRES;
double superanglei = 0.0;
void loop()
{
    debugPrint("loop core0 :");
    debugPrintln(millis());
    // toujours au meme endroit, le IS_OK(lidar.waitPoint()) crée un stack canary watchpoint trigger sur le core 0
    if (IS_OK(lidar.waitPoint()))
    {
        debugPrint(">point:");
        RPLIDARRES mesureRes = lidar.getCurrentPoint();
        if (mesureRes.distance < 1000)
        {
            float angle = mesureRes.angle * (float)DEG_TO_RAD;
            float distance = mesureRes.distance;
            float x = cos(angle) * distance;
            float y = sin(angle) * distance;
            debugPrint((double)x);
            debugPrint(":");
            debugPrint((double)y);
            debugPrint(";");
        }
        debugPrint((double)cos(superanglei) * 200);
        debugPrint(":");
        debugPrint((double)sin(superanglei) * 200);
        // debugPrint(":");
        // debugPrint((double)millis());
        debugPrintln("|xy");
        superanglei += 0.1;
        delay(50);
    }
    else
    {
        // analogWrite(D0, 0); // stop the rplidar motor
        debugPrintln("Lidar Stopped");
        // try to detect RPLIDAR...
        rplidar_response_device_info_t info;
        if (IS_OK(lidar.getDeviceInfo(info, 100)))
        {
            debugPrintln("Lidar found");
            // detected...
            lidar.startScan();
            // analogWrite(D0,150);
            delay(1000);
        }
        else
        {
            debugPrintln("Lidar not found");
        }
    }
    delay(25);
}
#elif HELLOWORLD == 2
#include <HCSR04.h>
HCSR04 hc(D7, D6);
void setup()
{
    Serial.begin(115200);
}
void loop()
{
    Serial.println(hc.dist());
    delay(500);
}
#elif HELLOWORLD == 3
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pcacard = Adafruit_PWMServoDriver();

#define SERVOMIN 125
#define SERVOMAX 575

void setup()
{
    Serial.begin(115200);

    pcacard.begin();
    pcacard.setPWMFreq(60);
}
int motori = 0;
int angleToPulse(int);
void loop()
{
    int angle = 45;
    pcacard.setPWM(14,0,angleToPulse(90-angle));
    pcacard.setPWM(15,0,angleToPulse(angle));
    // delay(50);
    // pcacard.setPWM(15, 0, angleToPulse(0));
    Serial.println("stop");
    delay(1000);
    // pcacard.setPWM(15,0,angleToPulse(20));
    // delay(1000);
}

int angleToPulse(int angle)
{
    int pulse = map(angle, 0, 90, SERVOMIN, SERVOMAX);
    return pulse;
}
#else
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

step_state etat_action = step_state::IDLE;

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
    etape{.action = atype::FORWARD, .distance = 1000000, .vitesse = 1000},
    // etape{.action = atype::TURN, .angle = 60, .vitesse = 1000},
    // etape{.action = atype::FORWARD, .distance = 100, .vitesse = 1000},
    // etape{.action = atype::TURN, .angle = 60, .vitesse = 1000},
    // etape{.action = atype::FORWARD, .distance = 100, .vitesse = 1000},
    // etape{.action = atype::TURN, .angle = 60, .vitesse = 1000}
};

strategie strat = stratun;

int etapeencour = -1;
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
    pinMode(Pin::Driver::EN, OUTPUT);
    digitalWrite(Pin::Driver::EN, LOW);
    etat_a = etat::INITALISATION;
    delay(1000);
}
bool showed_step = false;
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
        if (etapeencour >= stratun.size())
        { // etapesuivante =etapeencours + 1; par rapport à size() = etapesuivante -1; donc = etapeencour+1-1
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

#endif