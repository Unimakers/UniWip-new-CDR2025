//UniLidar by Aurélien
#include <lidar.h>
RPLidar lidar;
// HardwareSerial lidarSerial(1);
// HardwareSerial lidarSerial(1);
TaskHandle_t Task0;

float DIST_OBSTACLE = 350.0;
float MIN_DIST_OBSTACLE = 100.0;
int QUALITY = 14;
// int MEAN_ALLOWED = 2;

static portMUX_TYPE my_spinlock = portMUX_INITIALIZER_UNLOCKED;

bool lidarHasObstaclePiped = false;

bool lidarHasObstacle = false;

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
double clampAngle(double angle){
    return fmod((360+angle),360);
}
double calculateAngleFromDirection(DirectionVector v){
    return clampAngle((atan2(v.y,v.x)*RAD_TO_DEG)-90.0);
}


double currentDirectionAngle=0;

double angleInAngleRange(double angle_to_check){
    return true;
    return (angle_to_check)>=clampAngle(currentDirectionAngle-80) and (angle_to_check)<=clampAngle(currentDirectionAngle+80);
}
// bool angleInRange()
// {
//     return (mesure.angle >= 270 && mesure.angle <= 360) || (mesure.angle >= 120 && mesure.angle <= 180) || (mesure.angle >= 0 && mesure.angle <= 45);
// }
bool prevcdstate = false;
// FONOCTION COEUR 0 (COEUR LIDAR)
void checkAndSendObstacle(bool a){
    if(a){
        cooldownstarted=millis();
        cooldowning=true;
        prevcdstate=true;
        infoPrintln("hellolidarfuck");
        sendObstacleData(true);
        return;
    }

    if(cooldowning && abs((long)(millis()-cooldownstarted))<15000){
        // debugPrint("waiting at ");
        // debugPrintln(millis());
        sendObstacleData(true);
        return;
    }
    if(prevcdstate==true){
        prevcdstate=false;
        infoPrintln("disabling");
    }
    cooldowning=false;
    sendObstacleData(false);
}


int sumPoints = 0;
// FONCTION COEUR 0 (COUEUR LIDAR)
void get_point_lidar()
{
    if(lidarHasObstacle && (millis()-cooldownstarted)<3000){// && sumPoints>=15){
        lidarHasObstacle=true;
    }else{
        lidarHasObstacle=false;
        sumPoints=0;
    }
    // lidarHasObstacle=false;
    if (IS_OK(lidar.waitPoint(100)))
    {
        // bool supobstacleval = false;
        mesure.angle = lidar.getCurrentPoint().angle;

        // debugPrintln(((std::string)("a"+std::to_string(mesure.angle))).c_str());
        if (angleInAngleRange(mesure.angle))
        {
            mesure.distance = lidar.getCurrentPoint().distance; // distance value in mm unit
            mesure.quality = lidar.getCurrentPoint().quality;
            // debugPrintln(((std::string)("q"+std::to_string(mesure.quality))).c_str());
            if (mesure.quality >= QUALITY)
            {
                if (obstacle(mesure.distance))
                {
                    // debugPrintln(((std::string)("d"+std::to_string(mesure.distance)+" "+std::to_string(mesure.angle))).c_str());
                    // double xPoint = cos(mesure.angle) * mesure.distance;
                    // double yPoint = sin(mesure.angle) * mesure.distance;
                    // debugPrintln(((std::string) ">point:" + std::to_string(xPoint) + ":" + std::to_string(yPoint) + "|xy").c_str());
                    // debugPrintln(((std::string) "angle:" + std::to_string(mesure.angle)).c_str());
                    // supobstacleval=true;
                    // if(millis()-cooldownstarted<1000){
                    //     sumPoints++;
                    //     if(sumPoints>=15){
                            lidarHasObstacle=true;
                    //     }
                    // }else{
                    //     sumPoints=1;
                    // }
                    cooldownstarted=millis();
                }
                else{
                    // supobstacleval=false;
                }
            }
            else
            {
                reset_point();
            }
        }
        // checkAndSendObstacle(supobstacleval);
    }
    else
    {
        analogWrite(Pin::IHM::LIDAR_PWM, 0); // stop the rplidar motor
        // debugPrintln("Lidar Stopped");
        // try to detect RPLIDAR...
        rplidar_response_device_info_t info;
        if (IS_OK(lidar.getDeviceInfo(info, 100)))
        {
            debugPrintln("Lidar found");
            // detected...
            lidar.startScan();
            analogWrite(Pin::IHM::LIDAR_PWM, 160);
            delay(1000);
        }
        else
        {
            debugPrintln("Lidar not found");
        }
    }
    // delay(250);
}
#include <HCSR04.h>
HCSR04 hc(D7, D6);
void get_point_hcsr04(){
    float dist = hc.dist();
    debugPrintln(dist);
    // debugPrintln(hc.dist());
    if(dist>3 && dist<8){
        lidarHasObstacle=true;
        cooldownstarted=millis();
    }
    else if(lidarHasObstacle && (millis()-cooldownstarted)<3000){

    }else{
        lidarHasObstacle=false;
    }
    delay(100);
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
    if(pamimode){
        for(;;){
            get_point_hcsr04();
        }
    }
    else{
        pinMode(Pin::IHM::LIDAR_PWM, OUTPUT);
        analogWrite(Pin::IHM::LIDAR_PWM, 160);
        lidar.begin(Serial0);
        lidar.startScan();
        for (;;)
        {
            // debugPrintln("hello");
            get_point_lidar();
            
        }
    }
}

// FONCTION A EXECUTER COTE COEUR 1 (COEUR DE MOUVEMENT)
void initLidar()
{
    // if (!lidarInitialized)
    // {
    xTaskCreatePinnedToCore(LidarTask, "Task0", 10000, NULL, 0, NULL, 0);
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
bool* getLidarStatus()
{
    // taskENTER_CRITICAL(&my_spinlock);
    // lidarHasObstacle = lidarHasObstaclePiped;
    // taskEXIT_CRITICAL(&my_spinlock);
    // if(lidarHasObstacle){
    //     // debugPrint("lidar is here");
    //     // debugPrintln(millis());
    // }
    return &lidarHasObstacle;
}
void sendCurrentAngle(DirectionVector v){
    double angle = calculateAngleFromDirection(v);
    taskENTER_CRITICAL(&my_spinlock);
    currentDirectionAngle = angle;
    taskEXIT_CRITICAL(&my_spinlock);
}
// FONCTION COEUR 0 (COUEUR LIDAR)
bool Angle_in_range_scare()
{
    return (mesure.angle >= 300 && mesure.angle <= 360) and (mesure.distance < DIST_OBSTACLE / sin(mesure.angle));
}
// FONCTION COEUR 0 (COUEUR LIDAR)
bool obstacle(float distance)
{
    if (distance < DIST_OBSTACLE && distance > MIN_DIST_OBSTACLE)
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
