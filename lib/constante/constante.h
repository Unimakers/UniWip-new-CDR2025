#pragma once
#include <Arduino.h>
#define debugPrint Serial.print
#define debugPrintln Serial.println
#define infoPrintln Serial.println
#define infoPrint Serial.print
namespace Pin
{
    namespace Driver
    {
        constexpr int
            DIR_G = D0,//13,
            STEP_G = D1,
            DIR_D = D2,
            STEP_D = D3,
            EN = D8;
    } // namespace Driver

    namespace IHM
    {
        constexpr int
            TIRETTE = D10,
            LIDAR_PWM=D9,
            LIDAR_RX=RX,
            LIDAR_TX=TX;

    } // namespace IHM

} // namespace Pin
namespace Math
{
    constexpr double
        PI14 = 3.1415926535;
    constexpr int signint(int x){
        return (x > 0) - (x < 0);
    }
    constexpr double signum(double x){
        return (x > 0) - (x < 0);
    }
} // namespace math

namespace Physique
{
    constexpr double
        LARGEUR = 35.6,
        ECRT_ROUE = 26,
        DIAM_ROUE = 7.5,
        STEP_MULTI= 16,
        STEP_REV = 200*STEP_MULTI,
        STEP_CM = (STEP_REV / (2 * Math::PI14 * (DIAM_ROUE / 2))),
        ACCELARATION = 10;
} // namespace physique
