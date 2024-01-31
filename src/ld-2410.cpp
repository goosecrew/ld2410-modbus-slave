#include <Arduino.h>
#include <ld2410.h>
#include "./modbus-slave.h"

#define LD2410_TX 27
#define LD2410_RX 26
#define LD2410_SPEED 256000

ld2410 radar;
bool engineeringMode = false;
String command;


void radarSetMaxValues(uint16_t moving, uint16_t stationary, uint16_t inactivityTimer) {
  radar.setMaxValues(moving, stationary, inactivityTimer);
}


void pollLD2410() {

  radar.read();

  if(radar.isConnected()){
    setSENSOR_PRESENCE(radar.presenceDetected());
    setSENSOR_MOVING(radar.movingTargetDetected());
    setSENSOR_STATIONARY(radar.stationaryTargetDetected());
    setSENSOR_STATIONARY_TARGET_DISTANCE(radar.stationaryTargetDistance());
    setSENSOR_STATIONARY_TARGET_ENERGY(radar.stationaryTargetEnergy());
    setSENSOR_MOVING_TARGET_DISTANCE(radar.movingTargetDistance());
    setSENSOR_MOVING_TARGET_ENERGY(radar.movingTargetEnergy()); 
  }

  /*
  если подключиться к esp32 через miniUSB, то можно посылать команды и получать ответ с показаниями прям в консоль, команды:
  - read
  - readconfig
  - setmaxvalues
  - setsensitivity
  - enableengineeringmode
  - disableengineeringmode
  - restart
  - readversion
  - factoryreset
  */
  if(Serial.available())
  {
    char typedCharacter = Serial.read();
    if(typedCharacter == '\r' || typedCharacter == '\n')
    {
      command.trim();
      if(command.equals("read"))
      {
        command = "";
        Serial.print(F("Reading from sensor: "));
        if(radar.isConnected())
        {
          Serial.println(F("OK"));
          if(radar.presenceDetected())
          {
            if(radar.stationaryTargetDetected())
            {
              Serial.print(F("Stationary target: "));
              Serial.print(radar.stationaryTargetDistance());
              Serial.print(F("cm energy: "));
              Serial.println(radar.stationaryTargetEnergy());
            }
            if(radar.movingTargetDetected())
            {
              Serial.print(F("Moving target: "));
              Serial.print(radar.movingTargetDistance());
              Serial.print(F("cm energy: "));
              Serial.println(radar.movingTargetEnergy());
            }
          }
          else
          {
            Serial.println(F("nothing detected"));
          }
        }
        else
        {
          Serial.println(F("failed to read"));
        }
      }
      else if(command.equals("readconfig"))
      {
        command = "";
        Serial.print(F("Reading configuration from sensor: "));
        if(radar.requestCurrentConfiguration())
        {
          Serial.println(F("OK"));
          Serial.print(F("Maximum gate ID: "));
          Serial.println(radar.max_gate);
          Serial.print(F("Maximum gate for moving targets: "));
          Serial.println(radar.max_moving_gate);
          Serial.print(F("Maximum gate for stationary targets: "));
          Serial.println(radar.max_stationary_gate);
          Serial.print(F("Idle time for targets: "));
          Serial.println(radar.sensor_idle_time);
          Serial.println(F("Gate sensitivity"));
          for(uint8_t gate = 0; gate <= radar.max_gate; gate++)
          {
            Serial.print(F("Gate "));
            Serial.print(gate);
            Serial.print(F(" moving targets: "));
            Serial.print(radar.motion_sensitivity[gate]);
            Serial.print(F(" stationary targets: "));
            Serial.println(radar.stationary_sensitivity[gate]);
          }
        }
        else
        {
          Serial.println(F("Failed"));
        }
      }
      else if(command.startsWith("setmaxvalues"))
      {
        uint8_t firstSpace = command.indexOf(' ');
        uint8_t secondSpace = command.indexOf(' ',firstSpace + 1);
        uint8_t thirdSpace = command.indexOf(' ',secondSpace + 1);
        uint8_t newMovingMaxDistance = (command.substring(firstSpace,secondSpace)).toInt();
        uint8_t newStationaryMaxDistance = (command.substring(secondSpace,thirdSpace)).toInt();
        uint16_t inactivityTimer = (command.substring(thirdSpace,command.length())).toInt();
        if(newMovingMaxDistance > 0 && newStationaryMaxDistance > 0 && newMovingMaxDistance <= 8 && newStationaryMaxDistance <= 8)
        {
          Serial.print(F("Setting max values to gate "));
          Serial.print(newMovingMaxDistance);
          Serial.print(F(" moving targets, gate "));
          Serial.print(newStationaryMaxDistance);
          Serial.print(F(" stationary targets, "));
          Serial.print(inactivityTimer);
          Serial.print(F("s inactivity timer: "));
          command = "";
          if(radar.setMaxValues(newMovingMaxDistance, newStationaryMaxDistance, inactivityTimer))
          {
            Serial.println(F("OK, now restart to apply settings"));
          }
          else
          {
            Serial.println(F("failed"));
          }
        }
        else
        {
          Serial.print(F("Can't set distances to "));
          Serial.print(newMovingMaxDistance);
          Serial.print(F(" moving "));
          Serial.print(newStationaryMaxDistance);
          Serial.println(F(" stationary, try again"));
          command = "";
        }
      }
      else if(command.startsWith("setsensitivity"))
      {
        uint8_t firstSpace = command.indexOf(' ');
        uint8_t secondSpace = command.indexOf(' ',firstSpace + 1);
        uint8_t thirdSpace = command.indexOf(' ',secondSpace + 1);
        uint8_t gate = (command.substring(firstSpace,secondSpace)).toInt();
        uint8_t motionSensitivity = (command.substring(secondSpace,thirdSpace)).toInt();
        uint8_t stationarySensitivity = (command.substring(thirdSpace,command.length())).toInt();
        if(motionSensitivity >= 0 && stationarySensitivity >= 0 && motionSensitivity <= 100 && stationarySensitivity <= 100)
        {
          Serial.print(F("Setting gate "));
          Serial.print(gate);
          Serial.print(F(" motion sensitivity to "));
          Serial.print(motionSensitivity);
          Serial.print(F(" & stationary sensitivity to "));
          Serial.print(stationarySensitivity);
          Serial.println(F(": "));
          command = "";
          if(radar.setGateSensitivityThreshold(gate, motionSensitivity, stationarySensitivity))
          {
            Serial.println(F("OK, now restart to apply settings"));
          }
          else
          {
            Serial.println(F("failed"));
          }
        }
        else
        {
          Serial.print(F("Can't set gate "));
          Serial.print(gate);
          Serial.print(F(" motion sensitivity to "));
          Serial.print(motionSensitivity);
          Serial.print(F(" & stationary sensitivity to "));
          Serial.print(stationarySensitivity);
          Serial.println(F(", try again"));
          command = "";
        }
      }
      else if(command.equals("enableengineeringmode"))
      {
        command = "";
        Serial.print(F("Enabling engineering mode: "));
        if(radar.requestStartEngineeringMode())
        {
          Serial.println(F("OK"));
        }
        else
        {
          Serial.println(F("failed"));
        }
      }
      else if(command.equals("disableengineeringmode"))
      {
        command = "";
        Serial.print(F("Disabling engineering mode: "));
        if(radar.requestEndEngineeringMode())
        {
          Serial.println(F("OK"));
        }
        else
        {
          Serial.println(F("failed"));
        }
      }
      else if(command.equals("restart"))
      {
        command = "";
        Serial.print(F("Restarting sensor: "));
        if(radar.requestRestart())
        {
          Serial.println(F("OK"));
        }
        else
        {
          Serial.println(F("failed"));
        }
      }
      else if(command.equals("readversion"))
      {
        command = "";
        Serial.print(F("Requesting firmware version: "));
        if(radar.requestFirmwareVersion())
        {
          Serial.print('v');
          Serial.print(radar.firmware_major_version);
          Serial.print('.');
          Serial.print(radar.firmware_minor_version);
          Serial.print('.');
          Serial.println(radar.firmware_bugfix_version,HEX);
        }
        else
        {
          Serial.println(F("Failed"));
        }
      }
      else if(command.equals("factoryreset"))
      {
        command = "";
        Serial.print(F("Factory resetting sensor: "));
        if(radar.requestFactoryReset())
        {
          Serial.println(F("OK, now restart sensor to take effect"));
        }
        else
        {
          Serial.println(F("failed"));
        }
      }
      else
      {
        Serial.print(F("Unknown command: "));
        Serial.println(command);
        command = "";
      }
    }
    else
    {
      command += typedCharacter;
    }
  }


}



void ld2410_setup() {
  Serial2.begin(LD2410_SPEED, SERIAL_8N1, LD2410_RX, LD2410_TX);
  if(radar.begin(Serial2))
  {
    Serial.println(F("OK"));
  }
  else
  {
    Serial.println(F("not connected"));
  }

  radar.read();
  if(!radar.isConnected()){
    return;
  }
  if(!radar.requestCurrentConfiguration()) {
    return;
  }
  setSENSOR_MAX_MOVING_GATE(radar.max_moving_gate);
  setSENSOR_MAX_STATIONARY_GATE(radar.max_stationary_gate);
  setSENSOR_IDLE_TIME(radar.sensor_idle_time);
  Serial.printf("%d; %d; %d;\n", radar.max_moving_gate, radar.max_stationary_gate, radar.sensor_idle_time);
}