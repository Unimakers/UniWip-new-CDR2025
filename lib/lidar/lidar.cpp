#include <Arduino.h>
#include <constante.h>
#include <RPLidar.h>
#include <Wire.h>
#include <SPI.h>
#include <string>
RPLidar lidar;
HardwareSerial lidarSerial(1);
// HardwareSerial lidarSerial(1);
TaskHandle_t Task0;

int DIST_OBSTACLE = 550;
int QUALITY = 14;
int MEAN_ALLOWED = 2;

static portMUX_TYPE my_spinlock = portMUX_INITIALIZER_UNLOCKED;

bool lidarHasObstaclePiped = false;

bool lidarHasObstacle = false;
void sendObstacleData(bool);
bool obstacle();
void reset_point();
bool lidarInitialized = false;
long cooldownstarted = 0;
bool cooldowning = false;

// FONCTION COEUR 0 (COUEUR LIDAR)
typedef struct
{
    float distance = 0;
    float angle = 0;
    float quality = 0;
} LIDAR_MESURE_RES;

LIDAR_MESURE_RES mesure;
// FONCTION COEUR 0 (COUEUR LIDAR)
bool angleInRange()
{
    return (mesure.angle >= 270 && mesure.angle <= 360) || (mesure.angle >= 120 && mesure.angle <= 180) || (mesure.angle >= 0 && mesure.angle <= 45);
}
// FONOCTION COEUR 0 (COEUR LIDAR)
void checkAndSendObstacle(bool a){
    if(a){
        cooldownstarted=millis();
        cooldowning=true;
        sendObstacleData(true);
        return;
    }
    if(cooldowning && abs((long)(millis()-cooldownstarted))<10000) return;
    cooldowning=false;
    sendObstacleData(false);
}
// FONCTION COEUR 0 (COUEUR LIDAR)
void get_point_lidar()
{
    if (IS_OK(lidar.waitPoint()))
    {
        mesure.angle = lidar.getCurrentPoint().angle;
        if (angleInRange())
        {
            mesure.distance = lidar.getCurrentPoint().distance; // distance value in mm unit
            mesure.quality = lidar.getCurrentPoint().quality;
            if (mesure.quality > QUALITY)
            {
                if (obstacle())
                {
                    // double xPoint = cos(mesure.angle) * mesure.distance;
                    // double yPoint = sin(mesure.angle) * mesure.distance;
                    // Serial.println(((std::string) ">point:" + std::to_string(xPoint) + ":" + std::to_string(yPoint) + "|xy").c_str());
                    checkAndSendObstacle(true);
                }
                else{
                    checkAndSendObstacle(false);
                }
            }
            else
            {
                reset_point();
            }
        }
        delay(250);
    }
    else
    {
        // analogWrite(PIN::LIDAR::PWM, 0); // stop the rplidar motor
        // Serial.println("Lidar Stopped");
        // try to detect RPLIDAR...
        rplidar_response_device_info_t info;
        if (IS_OK(lidar.getDeviceInfo(info, 100)))
        {
            Serial.println("Lidar found");
            // detected...
            lidar.startScan();
            analogWrite(Pin::IHM::LIDAR_PWM, 150);
            delay(1000);
        }
        else
        {
            Serial.println("Lidar not found");
        }
    }
    delay(25);
}
// FONCTION COEUR 0 (COUEUR LIDAR)
void LidarTask(void *pvParameters)
{
    // if (!lidarInitialized)
    // {

    //     // taskENTER_CRITICAL(&my_spinlock);
    //     // lidarInitialized = true;
    //     // taskEXIT_CRITICAL(&my_spinlock);
    // }
    lidar.begin(lidarSerial, Pin::IHM::LIDAR_RX, Pin::IHM::LIDAR_TX);
    lidar.startScan();
    for (;;)
    {
        // Serial.println("hello");
        get_point_lidar();
        delay(250);
    }
}

// FONCTION A EXECUTER COTE COEUR 1 (COEUR DE MOUVEMENT)
void initLidar()
{
    // if (!lidarInitialized)
    // {
    pinMode(Pin::IHM::LIDAR_PWM, OUTPUT);
    analogWrite(Pin::IHM::LIDAR_PWM, 150);
    xTaskCreatePinnedToCore(LidarTask, "Task0", 10000, NULL, 1, &Task0, 0);
    // }
}

// FONCTION COEUR 0 (COUEUR LIDAR)
void sendObstacleData(bool obstacleVal)
{
    taskENTER_CRITICAL(&my_spinlock);
    lidarHasObstaclePiped = obstacleVal;
    taskEXIT_CRITICAL(&my_spinlock);
}

// FONCTION A EXECUTER COTE COEUR 1 (COEUR DE MOUVEMENT)
bool getLidarStatus()
{
    taskENTER_CRITICAL(&my_spinlock);
    lidarHasObstacle = lidarHasObstaclePiped;
    taskEXIT_CRITICAL(&my_spinlock);
    return lidarHasObstacle;
}
// FONCTION COEUR 0 (COUEUR LIDAR)
bool Angle_in_range_scare()
{
    return (mesure.angle >= 300 && mesure.angle <= 360) and (mesure.distance < DIST_OBSTACLE / sin(mesure.angle));
}
// FONCTION COEUR 0 (COUEUR LIDAR)
bool obstacle()
{
    if (mesure.distance < DIST_OBSTACLE && mesure.distance > 10)
    {
        return true;
    }
    else
    {
        return false;
    }
}
// FONCTION COEUR 0 (COUEUR LIDAR)
void reset_point()
{
    mesure.distance = 0;
    mesure.angle = 0;
    mesure.quality = 0;
}
