#include <Arduino.h>
#include <Robotmove.h>
#include <PCF8574.h>

RobotMove robot;
enum struct etat
{
  PRE_INIT,
  INITALISATION,
  MATCH,
  FIN
};
etat etat_a = etat::PRE_INIT;
//PCF8574 pcf();

void setup()
{
  pinMode(Pin::IHM::TIRETTE, INPUT_PULLUP);
  // fonction de pre_init
  while (digitalRead(Pin::IHM::TIRETTE))
  {
    Serial.println("Mais t'es pas là mais t'es où?");
  }
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
    }
    else
    {
      etat_a = etat::MATCH;
      Serial.println("fin setup");
    }

    return;
  }
}
