//Include necessary libraries for compilation
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <MCP2515.h>
#include <mcp2515_defs.h>
#include <defaults.h>
#include <global.h>
#include <CAN.h>
#define CAN_2515

//Initialize uSD pins
const int csSD = 9;
const float pi = 3.14159;
const int csCANChinaMod = 3;  //CAN ChinaMod = modded DiyMore shield
const int irqCANChinaMod = 2;
const int csCANKeyes = 10;  //Keyestudio Shield (with SD Card)
const int irqCANKeyes = 8;
int sniffs = 0;
unsigned long current; //battery current
unsigned long accPos; //accelerator pedal position
float brakeValue; //brake pressure
float BatteryTemp; //battery cell temperature
float MotorCurrentCalc; //calculated motor amps from motor torque and RPM
float BatteryPower; //Calculated battery power in (+) or out (-) from volts and amps
float MotorPowerEV;
float MotorPowerNominal; //motor power using nominal torque from Car CAN
float MotorPowerActual; //motor power using actual torque from Car CAN
unsigned long currentEVSum;
byte ReadHiCar;
byte ReadLoCar;
byte ReadHiEV;
byte ReadLoEV;
byte ReadHiMotorEV;
byte ReadLoMotorEV;
unsigned long MotorVoltage; //inverter voltage from EV CAN
unsigned short ReadKmCar; //odometer reading
byte ReadHiMotorRPM;
byte ReadLoMotorRPM;
short ReadMotorRPM; //motor RPM from EV CAN
float ReadCurrent;
float motorAmps;
short ReadMotorTorque;
float ReadMotorTorque2; //Motor torque from EV CAN
float ReadSpeedCar; //vehicle speed from Car CAN
unsigned int SOC;
byte ReadHiVoltsEV;
byte ReadLoVoltsEV;
unsigned int ReadVolts;
float ReadCurrentCar;
float ReadNominalTorqueCar; //Nominal torque from Car CAN
float ReadActualTorqueCar; //Actual torque from Car CAN
float ReadBrakePedalCar;
const unsigned long writePeriod = 200;
const unsigned long timeOut = writePeriod * 2;
unsigned long prevCount;
unsigned long writeTimeOld = 0;
unsigned long startTime;

MCP2515Class CANCar;
MCP2515Class CANEV;


//Declare SD File
File dataFile;

void setup() {
  //Initialize Serial communication for debugging
  Serial.begin(38400);
  Serial.println("LEAF ZE-0 Logger");


  //Initialize pins as necessary
  pinMode(csSD, OUTPUT);

  //Initialise EVCAN (DIYMore)
  CANEV.setPins(csCANChinaMod, irqCANChinaMod);
  delay(100);
  if (CANEV.begin(500E3)) {
    Serial.println("CAN EV Init Ok");
    delay(100);
  } else {
    Serial.println("Can't init CAN EV");
  }

  //Initialize Car CAN (Keyestudio)
  CANCar.setPins(csCANKeyes, irqCANKeyes);
  if (CANCar.begin(500E3)) {
    Serial.println("CAN Car Init Ok");
    delay(100);
  } else {
    Serial.println("Can't init CAN Car");
  }

  //Check if uSD card initialized
  if (!SD.begin(csSD)) {
    Serial.println("uSD card failed to initialize, or is not present");
  } else {
    Serial.println("uSD card initialized.");
    delay(100);
  }

  delay(1000);
  pinMode(4, OUTPUT);
  CANEV.filter(0x1DA, 0xFFE);  //filter EVCAN to 1da and 1db messages

  File dataFile = SD.open("L231112A.txt", FILE_WRITE);
  if (!dataFile) {

  } else {
    //Here printing the header for each message
    dataFile.print("EV");
    dataFile.print("\t");
    dataFile.print("Time (ms)");
    dataFile.print("\t");
    dataFile.print("Mileage (km)");
    dataFile.print("\t");
    dataFile.print("1db");
    dataFile.print("\t");
    dataFile.print("High byte battery current EVCAN (HEX)");
    dataFile.print("\t");
    dataFile.print("Low byte battery current EVCAN (HEX)");
    dataFile.print("\t");
    dataFile.print("Battery current EVCAN two's complement decimal");
    dataFile.print("\t");
    dataFile.print("High byte battery voltage EVCAN (HEX)");
    dataFile.print("\t");
    dataFile.print("Low byte battery voltage EVCAN (HEX)");
    dataFile.print("\t");
    dataFile.println("battery voltage EVCAN decimal");
    dataFile.print("\t");
    dataFile.println("battery SOC EVCAN %");

    dataFile.print("EV");
    dataFile.print("\t");
    dataFile.print("Time (ms)");
    dataFile.print("\t");
    dataFile.print("Mileage (km)");
    dataFile.print("\t");
    dataFile.print("1da");
    dataFile.print("\t");
    dataFile.print("High byte motor torque EVCAN (HEX)");
    dataFile.print("\t");
    dataFile.print("Low byte motor torque EVCAN (HEX)");
    dataFile.print("\t");
    dataFile.print("Motor torque EVCAN two's complement decimal");
    dataFile.print("\t");
    dataFile.print("Inverter voltage");
    dataFile.print("\t");
    dataFile.print("Motor RPM Decimal");
    dataFile.print("\t");
    dataFile.println("Motor current calculated from torque, inverter voltage and RPM");

    dataFile.print("CAR");
    dataFile.print("\t");
    dataFile.print("Time (ms)");
    dataFile.print("\t");
    dataFile.print("Mileage (km)");
    dataFile.print("\t");
    dataFile.print("Battery current (A)");
    dataFile.print("\t");
    dataFile.print("Battery temp (C)");
    dataFile.print("\t");
    dataFile.print("Accelerator pedal position");
    dataFile.print("\t");
    dataFile.print("Brake pedal position");
    dataFile.print("\t");
    dataFile.print("Brake pressure master cylinder");
    dataFile.print("\t");
    dataFile.print("Vehicle speed (km/h)");
    dataFile.print("\t");
    dataFile.print("Nominal torque value (Nm)");
    dataFile.print("\t");
    dataFile.print("Actual engine value (Nm)");
    dataFile.print("\t");
    dataFile.print("Battery Power Calculated EVCAN");
    dataFile.print("\t");
    dataFile.print("Motor Power Calculated EVCAN");
    dataFile.print("\t");
    dataFile.print("Motor Power Calculated CAR Nominal");
    dataFile.print("\t");
    dataFile.print("Motor Power Calculated CAR Actual");
    dataFile.print("\t");
    dataFile.println("No. of sniffs EVCAN");

    dataFile.flush();
    dataFile.close();  //Close data logging file
  }
}

void readEV() {
  if (CANEV.parsePacket() > 0) {
    if (CANEV.packetId() == 0x1db) {
      ReadHiEV = CANEV.read();
      ReadLoEV = CANEV.read();
      ReadCurrent = float((ReadHiEV << 8) | ReadLoEV);
      ReadCurrent = float(ReadCurrent / 32);
      ReadCurrent = ReadCurrent / 2;

      ReadHiVoltsEV = CANEV.read();
      ReadLoVoltsEV = CANEV.read();
      ReadVolts = int((ReadHiVoltsEV << 8) | ReadLoVoltsEV);
      ReadVolts = (ReadVolts >> 6);
      ReadVolts = ReadVolts / 2;

      SOC = CANEV.read();
      sniffs++;
      //If data file can't be opened, throw error.
      if (!dataFile) {
        Serial.println("Cannot save EV");
      }

      else {
        dataFile.print("EV");
        dataFile.print("\t");
        dataFile.print(millis());
        dataFile.print("\t");
        dataFile.print(ReadKmCar);
        dataFile.print("\t");
        dataFile.print("1db");
        dataFile.print("\t");
        dataFile.print(ReadHiEV, HEX);
        dataFile.print("\t");
        dataFile.print(ReadLoEV, HEX);
        dataFile.print("\t");
        dataFile.print(ReadCurrent);
        dataFile.print("\t");
        dataFile.print(ReadHiVoltsEV, HEX);
        dataFile.print("\t");
        dataFile.print(ReadLoVoltsEV, HEX);
        dataFile.print("\t");
        dataFile.print(ReadVolts);
        dataFile.print("\t");
        dataFile.println(SOC);
      }
    } else if (CANEV.packetId() == 0x1da) {
      MotorVoltage = (unsigned long)CANEV.read();
      MotorVoltage = MotorVoltage * 2;

      CANEV.read();

      ReadHiMotorEV = CANEV.read();
      ReadLoMotorEV = CANEV.read();
      ReadHiMotorEV = byte(ReadHiMotorEV & 0b00000111);
      ReadMotorTorque = short(((ReadHiMotorEV << 8) | ReadLoMotorEV) << 5);
      ReadMotorTorque2 = ReadMotorTorque / 32;
      ReadMotorTorque2 = ReadMotorTorque2 / 2;

      ReadHiMotorRPM = CANEV.read();
      ReadLoMotorRPM = CANEV.read();
      ReadMotorRPM = short((ReadHiMotorRPM << 8) | ReadLoMotorRPM);
      ReadMotorRPM = ReadMotorRPM / 2;

      MotorCurrentCalc = (ReadMotorTorque2 * ReadMotorRPM * 2 * pi) / (60 * ReadVolts);

      if (!dataFile) {
        Serial.println("Cannot save");
      }

      else {
        dataFile.print("EV");
        dataFile.print("\t");
        dataFile.print(millis());
        dataFile.print("\t");
        dataFile.print(ReadKmCar);
        dataFile.print("\t");
        dataFile.print("1da");
        dataFile.print("\t");
        dataFile.print(ReadHiMotorEV, HEX);
        dataFile.print("\t");
        dataFile.print(ReadLoMotorEV, HEX);
        dataFile.print("\t");
        dataFile.print(ReadMotorTorque2);
        dataFile.print("\t");
        dataFile.print(MotorVoltage);
        dataFile.print("\t");
        dataFile.print(ReadMotorRPM);
        dataFile.print("\t");
        dataFile.println(MotorCurrentCalc);
      }
    }
  } else {
  }
}


//********************************Main Loop*********************************//
void loop() {

  if ((millis() - prevCount) > 10000) {
    CANEV.filter(0x1DA, 0xFFE);
    prevCount = millis();
  }

  File dataFile = SD.open("L231112A.txt", FILE_WRITE);
  while (writePeriod > millis() - writeTimeOld) {  //on first iteration, this while will not go in because there are long delays in setup, this while waits until its time to write another question.
                                                   //This writePeriod sets the time between writes to SD card and serial monitor, not rate of CAN message write.

    if (CANEV.parsePacket() > 0) {
      if (CANEV.packetId() == 0x1db) {
        ReadHiEV = CANEV.read();
        ReadLoEV = CANEV.read();
        ReadCurrent = float((ReadHiEV << 8) | ReadLoEV);
        ReadCurrent = float(ReadCurrent / 32);
        ReadCurrent = ReadCurrent / 2;

        ReadHiVoltsEV = CANEV.read();
        ReadLoVoltsEV = CANEV.read();
        ReadVolts = int((ReadHiVoltsEV << 8) | ReadLoVoltsEV);
        ReadVolts = (ReadVolts >> 6);
        ReadVolts = ReadVolts / 2;

        SOC = CANEV.read();
        sniffs++;

        //If data file can't be opened, throw error.
        if (!dataFile) {
          Serial.println("Cannot save EV");
        }

        else {
          dataFile.print("EV");
          dataFile.print("\t");
          dataFile.print(millis());
          dataFile.print("\t");
          dataFile.print(ReadKmCar);
          dataFile.print("\t");
          dataFile.print("1db");
          dataFile.print("\t");
          dataFile.print(ReadHiEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadLoEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadCurrent);
          dataFile.print("\t");
          dataFile.print(ReadHiVoltsEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadLoVoltsEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadVolts);
          dataFile.print("\t");
          dataFile.println(SOC);
        }
      } else if (CANEV.packetId() == 0x1da) {
        MotorVoltage = (unsigned long)CANEV.read();
        MotorVoltage = MotorVoltage * 2;

        CANEV.read();

        ReadHiMotorEV = CANEV.read();
        ReadLoMotorEV = CANEV.read();
        ReadHiMotorEV = byte(ReadHiMotorEV & 0b00000111);
        ReadMotorTorque = short(((ReadHiMotorEV << 8) | ReadLoMotorEV) << 5);
        ReadMotorTorque2 = ReadMotorTorque / 32;
        ReadMotorTorque2 = ReadMotorTorque2 / 2;

        ReadHiMotorRPM = CANEV.read();
        ReadLoMotorRPM = CANEV.read();
        ReadMotorRPM = short((ReadHiMotorRPM << 8) | ReadLoMotorRPM);
        ReadMotorRPM = ReadMotorRPM / 2;
        MotorCurrentCalc = (ReadMotorTorque2 * ReadMotorRPM * 2 * pi) / (60 * ReadVolts);  //Calculated amps from torque

        if (!dataFile) {
        }

        else {
          dataFile.print("EV");
          dataFile.print("\t");
          dataFile.print(millis());
          dataFile.print("\t");
          dataFile.print(ReadKmCar);
          dataFile.print("\t");
          dataFile.print("1da");
          dataFile.print("\t");
          dataFile.print(ReadHiMotorEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadLoMotorEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadMotorTorque2);
          dataFile.print("\t");
          dataFile.print(MotorVoltage);
          dataFile.print("\t");
          dataFile.print(ReadMotorRPM);
          dataFile.print("\t");
          dataFile.println(MotorCurrentCalc);
        }
      }
    } else {
    }
  }

  //Here it is assumed that between reading the previous reply to writing the next question there is too little time to capture any sniffs on EV CAN.
  digitalWrite(4, HIGH);
  CANCar.filter(0x79A);
  CANCar.beginPacket(0x797, 8);
  CANCar.write(0x03);
  CANCar.write(0x22);
  CANCar.write(0x12);
  CANCar.write(0x48);
  CANCar.endPacket();

  writeTimeOld = millis();
  startTime = millis();
  //Serial.println("Writing to Car CAN ");



  while ((CANCar.parsePacket() == 0 ||  //Waiting until reply comes from CarCAN
          CANCar.read() < 5 || CANCar.read() != 0x62 || CANCar.read() != 0x12 || CANCar.read() != 0x48)
         && timeOut > millis() - startTime) {  

    if (CANEV.parsePacket() > 0) {
      if (CANEV.packetId() == 0x1db) {
        ReadHiEV = CANEV.read();
        ReadLoEV = CANEV.read();
        ReadCurrent = float((ReadHiEV << 8) | ReadLoEV);
        ReadCurrent = float(ReadCurrent / 32);
        ReadCurrent = ReadCurrent / 2;

        ReadHiVoltsEV = CANEV.read();
        ReadLoVoltsEV = CANEV.read();
        ReadVolts = int((ReadHiVoltsEV << 8) | ReadLoVoltsEV);
        ReadVolts = (ReadVolts >> 6);
        ReadVolts = ReadVolts / 2;

        SOC = CANEV.read();
        sniffs++;

        //If data file can't be opened, throw error.
        if (!dataFile) {
          Serial.println("Cannot save EV");
        }

        else {
          dataFile.print("EV");
          dataFile.print("\t");
          dataFile.print(millis());
          dataFile.print("\t");
          dataFile.print(ReadKmCar);
          dataFile.print("\t");
          dataFile.print("1db");
          dataFile.print("\t");
          dataFile.print(ReadHiEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadLoEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadCurrent);
          dataFile.print("\t");
          dataFile.print(ReadHiVoltsEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadLoVoltsEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadVolts);
          dataFile.print("\t");
          dataFile.println(SOC);
        }
      } else if (CANEV.packetId() == 0x1da) {
        MotorVoltage = (unsigned long)CANEV.read();
        MotorVoltage = MotorVoltage * 2;

        CANEV.read();

        ReadHiMotorEV = CANEV.read();
        ReadLoMotorEV = CANEV.read();
        ReadHiMotorEV = byte(ReadHiMotorEV & 0b00000111);
        ReadMotorTorque = short(((ReadHiMotorEV << 8) | ReadLoMotorEV) << 5);
        ReadMotorTorque2 = ReadMotorTorque / 32;
        ReadMotorTorque2 = ReadMotorTorque2 / 2;

        ReadHiMotorRPM = CANEV.read();
        ReadLoMotorRPM = CANEV.read();
        ReadMotorRPM = short((ReadHiMotorRPM << 8) | ReadLoMotorRPM);
        ReadMotorRPM = ReadMotorRPM / 2;

        MotorCurrentCalc = (ReadMotorTorque2 * ReadMotorRPM * 2 * pi) / (60 * ReadVolts);

        if (!dataFile) {

        }

        else {
          dataFile.print("EV");
          dataFile.print("\t");
          dataFile.print(millis());
          dataFile.print("\t");
          dataFile.print(ReadKmCar);
          dataFile.print("\t");
          dataFile.print("1da");
          dataFile.print("\t");
          dataFile.print(ReadHiMotorEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadLoMotorEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadMotorTorque2);
          dataFile.print("\t");
          dataFile.print(MotorVoltage);
          dataFile.print("\t");
          dataFile.print(ReadMotorRPM);
          dataFile.print("\t");
          dataFile.println(MotorCurrentCalc);
        }
      }
    } else {
    }
  }
  if (timeOut > (unsigned long)(millis() - startTime)) {
    ReadHiCar = CANCar.read();
    ReadLoCar = CANCar.read();
    ReadCurrentCar = (ReadHiCar << 8) | ReadLoCar;
    ReadCurrentCar = ReadCurrentCar / 2;
  }

  digitalWrite(4, HIGH);
  CANCar.filter(0x79A);
  CANCar.beginPacket(0x797, 8);
  CANCar.write(0x03);
  CANCar.write(0x22);
  CANCar.write(0x11);
  CANCar.write(0x02);
  CANCar.endPacket();

  writeTimeOld = millis();
  startTime = millis();

  while ((CANCar.parsePacket() == 0 ||  //Waiting until reply comes from CarCAN
          CANCar.read() < 4 || CANCar.read() != 0x62 || CANCar.read() != 0x11 || CANCar.read() != 0x02)
         && timeOut > millis() - startTime) {

    if (CANEV.parsePacket() > 0) {
      if (CANEV.packetId() == 0x1db) {
        ReadHiEV = CANEV.read();
        ReadLoEV = CANEV.read();
        ReadCurrent = float((ReadHiEV << 8) | ReadLoEV);
        ReadCurrent = float(ReadCurrent / 32);
        ReadCurrent = ReadCurrent / 2;

        ReadHiVoltsEV = CANEV.read();
        ReadLoVoltsEV = CANEV.read();
        ReadVolts = int((ReadHiVoltsEV << 8) | ReadLoVoltsEV);
        ReadVolts = (ReadVolts >> 6);
        ReadVolts = ReadVolts / 2;

        SOC = CANEV.read();
        sniffs++;

        //If data file can't be opened, throw error.
        if (!dataFile) {
          Serial.println("Cannot save EV");
        }

        else {
          dataFile.print("EV");
          dataFile.print("\t");
          dataFile.print(millis());
          dataFile.print("\t");
          dataFile.print(ReadKmCar);
          dataFile.print("\t");
          dataFile.print("1db");
          dataFile.print("\t");
          dataFile.print(ReadHiEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadLoEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadCurrent);
          dataFile.print("\t");
          dataFile.print(ReadHiVoltsEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadLoVoltsEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadVolts);
          dataFile.print("\t");
          dataFile.println(SOC);
        }
      } else if (CANEV.packetId() == 0x1da) {
        MotorVoltage = (unsigned long)CANEV.read();
        MotorVoltage = MotorVoltage * 2;

        CANEV.read();

        ReadHiMotorEV = CANEV.read();
        ReadLoMotorEV = CANEV.read();
        ReadHiMotorEV = byte(ReadHiMotorEV & 0b00000111);
        ReadMotorTorque = short(((ReadHiMotorEV << 8) | ReadLoMotorEV) << 5);
        ReadMotorTorque2 = ReadMotorTorque / 32;
        ReadMotorTorque2 = ReadMotorTorque2 / 2;

        ReadHiMotorRPM = CANEV.read();
        ReadLoMotorRPM = CANEV.read();
        ReadMotorRPM = short((ReadHiMotorRPM << 8) | ReadLoMotorRPM);
        ReadMotorRPM = ReadMotorRPM / 2;

        MotorCurrentCalc = (ReadMotorTorque2 * ReadMotorRPM * 2 * pi) / (60 * ReadVolts);

        if (!dataFile) {
        }

        else {
          dataFile.print("EV");
          dataFile.print("\t");
          dataFile.print(millis());
          dataFile.print("\t");
          dataFile.print(ReadKmCar);
          dataFile.print("\t");
          dataFile.print("1da");
          dataFile.print("\t");
          dataFile.print(ReadHiMotorEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadLoMotorEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadMotorTorque2);
          dataFile.print("\t");
          dataFile.print(MotorVoltage);
          dataFile.print("\t");
          dataFile.print(ReadMotorRPM);
          dataFile.print("\t");
          dataFile.println(MotorCurrentCalc);
        }
      }
    } else {
    }
  }
  if (timeOut > (unsigned long)(millis() - startTime)) {
    ReadSpeedCar = CANCar.read();
  }

  digitalWrite(4, HIGH);
  CANCar.filter(0x7BB);
  CANCar.beginPacket(0x79B, 8);
  CANCar.write(0x02);
  CANCar.write(0x21);
  CANCar.write(0x04);
  CANCar.endPacket();

  writeTimeOld = millis();
  startTime = millis();

  while ((CANCar.parsePacket() == 0 ||  //Waiting until reply comes from CarCAN
          CANCar.read() < 10 || CANCar.read() != 0x10 || CANCar.read() != 0x61 || CANCar.read() != 0x04)
         && timeOut > millis() - startTime) { 
    if (CANEV.parsePacket() > 0) {
      if (CANEV.packetId() == 0x1db) {
        ReadHiEV = CANEV.read();
        ReadLoEV = CANEV.read();
        ReadCurrent = float((ReadHiEV << 8) | ReadLoEV);
        ReadCurrent = float(ReadCurrent / 32);
        ReadCurrent = ReadCurrent / 2;

        ReadHiVoltsEV = CANEV.read();
        ReadLoVoltsEV = CANEV.read();
        ReadVolts = int((ReadHiVoltsEV << 8) | ReadLoVoltsEV);
        ReadVolts = (ReadVolts >> 6);
        ReadVolts = ReadVolts / 2;

        SOC = CANEV.read();
        sniffs++;

        //If data file can't be opened, throw error.
        if (!dataFile) {
          Serial.println("Cannot save EV");
        }

        else {
          dataFile.print("EV");
          dataFile.print("\t");
          dataFile.print(millis());
          dataFile.print("\t");
          dataFile.print(ReadKmCar);
          dataFile.print("\t");
          dataFile.print("1db");
          dataFile.print("\t");
          dataFile.print(ReadHiEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadLoEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadCurrent);
          dataFile.print("\t");
          dataFile.print(ReadHiVoltsEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadLoVoltsEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadVolts);
          dataFile.print("\t");
          dataFile.println(SOC);
        }
      } else if (CANEV.packetId() == 0x1da) {
        MotorVoltage = (unsigned long)CANEV.read();
        MotorVoltage = MotorVoltage * 2;

        CANEV.read();

        ReadHiMotorEV = CANEV.read();
        ReadLoMotorEV = CANEV.read();
        ReadHiMotorEV = byte(ReadHiMotorEV & 0b00000111);
        ReadMotorTorque = short(((ReadHiMotorEV << 8) | ReadLoMotorEV) << 5);
        ReadMotorTorque2 = ReadMotorTorque / 32;
        ReadMotorTorque2 = ReadMotorTorque2 / 2;

        ReadHiMotorRPM = CANEV.read();
        ReadLoMotorRPM = CANEV.read();
        ReadMotorRPM = short((ReadHiMotorRPM << 8) | ReadLoMotorRPM);
        ReadMotorRPM = ReadMotorRPM / 2;

        MotorCurrentCalc = (ReadMotorTorque2 * ReadMotorRPM * 2 * pi) / (60 * ReadVolts);

        if (!dataFile) {

        }

        else {
          dataFile.print("EV");
          dataFile.print("\t");
          dataFile.print(millis());
          dataFile.print("\t");
          dataFile.print(ReadKmCar);
          dataFile.print("\t");
          dataFile.print("1da");
          dataFile.print("\t");
          dataFile.print(ReadHiMotorEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadLoMotorEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadMotorTorque2);
          dataFile.print("\t");
          dataFile.print(MotorVoltage);
          dataFile.print("\t");
          dataFile.print(ReadMotorRPM);
          dataFile.print("\t");
          dataFile.println(MotorCurrentCalc);
        }
      }
    } else {
    }
  }
  if (timeOut > (unsigned long)(millis() - startTime)) {
    CANCar.read();
    CANCar.read();
    BatteryTemp = float(CANCar.read());

    CANCar.beginPacket(0x79B, 8);
    CANCar.write(0x30);
    CANCar.endPacket();
  }

  digitalWrite(4, HIGH);
  CANCar.filter(0x763);
  CANCar.beginPacket(0x743, 8);
  CANCar.write(0x02);
  CANCar.write(0x21);
  CANCar.write(0x01);
  CANCar.endPacket();

  writeTimeOld = millis();
  startTime = millis();

  while ((CANCar.parsePacket() == 0 ||  //Waiting until reply comes from CarCAN
          CANCar.read() != 0x10 || CANCar.read() != 0x82 || CANCar.read() != 0x61 || CANCar.read() != 0x01)
         && timeOut > millis() - startTime) {
    if (CANEV.parsePacket() > 0) {
      if (CANEV.packetId() == 0x1db) {
        ReadHiEV = CANEV.read();
        ReadLoEV = CANEV.read();
        ReadCurrent = float((ReadHiEV << 8) | ReadLoEV);
        ReadCurrent = float(ReadCurrent / 32);
        ReadCurrent = ReadCurrent / 2;

        ReadHiVoltsEV = CANEV.read();
        ReadLoVoltsEV = CANEV.read();
        ReadVolts = int((ReadHiVoltsEV << 8) | ReadLoVoltsEV);
        ReadVolts = (ReadVolts >> 6);
        ReadVolts = ReadVolts / 2;

        SOC = CANEV.read();
        sniffs++;
        //If data file can't be opened, throw error.
        if (!dataFile) {
          Serial.println("Cannot save EV");
        }

        else {
          dataFile.print("EV");
          dataFile.print("\t");
          dataFile.print(millis());
          dataFile.print("\t");
          dataFile.print(ReadKmCar);
          dataFile.print("\t");
          dataFile.print("1db");
          dataFile.print("\t");
          dataFile.print(ReadHiEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadLoEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadCurrent);
          dataFile.print("\t");
          dataFile.print(ReadHiVoltsEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadLoVoltsEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadVolts);
          dataFile.print("\t");
          dataFile.println(SOC);
        }
      } else if (CANEV.packetId() == 0x1da) {
        MotorVoltage = (unsigned long)CANEV.read();
        MotorVoltage = MotorVoltage * 2;

        CANEV.read();

        ReadHiMotorEV = CANEV.read();
        ReadLoMotorEV = CANEV.read();
        ReadHiMotorEV = byte(ReadHiMotorEV & 0b00000111);
        ReadMotorTorque = short(((ReadHiMotorEV << 8) | ReadLoMotorEV) << 5);
        ReadMotorTorque2 = ReadMotorTorque / 32;
        ReadMotorTorque2 = ReadMotorTorque2 / 2;

        ReadHiMotorRPM = CANEV.read();
        ReadLoMotorRPM = CANEV.read();
        ReadMotorRPM = short((ReadHiMotorRPM << 8) | ReadLoMotorRPM);
        ReadMotorRPM = ReadMotorRPM / 2;

        MotorCurrentCalc = (ReadMotorTorque2 * ReadMotorRPM * 2 * pi) / (60 * ReadVolts);

        if (!dataFile) {
          
        }

        else {
          dataFile.print("EV");
          dataFile.print("\t");
          dataFile.print(millis());
          dataFile.print("\t");
          dataFile.print(ReadKmCar);
          dataFile.print("\t");
          dataFile.print("1da");
          dataFile.print("\t");
          dataFile.print(ReadHiMotorEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadLoMotorEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadMotorTorque2);
          dataFile.print("\t");
          dataFile.print(MotorVoltage);
          dataFile.print("\t");
          dataFile.print(ReadMotorRPM);
          dataFile.print("\t");
          dataFile.println(MotorCurrentCalc);
        }
      }
    } else {
    }
  }
  if (timeOut > (unsigned long)(millis() - startTime)) {
    CANCar.beginPacket(0x743, 8);
    CANCar.write(0x30);
    CANCar.endPacket();

    writeTimeOld = millis();
    startTime = millis();

    while ((CANCar.parsePacket() == 0 ||  //Waiting until reply comes from CarCAN
            CANCar.read() != 0x21)
           && timeOut > millis() - startTime) { 
      if (CANEV.parsePacket() > 0) {
        if (CANEV.packetId() == 0x1db) {
          ReadHiEV = CANEV.read();
          ReadLoEV = CANEV.read();
          ReadCurrent = float((ReadHiEV << 8) | ReadLoEV);
          ReadCurrent = float(ReadCurrent / 32);
          ReadCurrent = ReadCurrent / 2;

          ReadHiVoltsEV = CANEV.read();
          ReadLoVoltsEV = CANEV.read();
          ReadVolts = int((ReadHiVoltsEV << 8) | ReadLoVoltsEV);
          ReadVolts = (ReadVolts >> 6);
          ReadVolts = ReadVolts / 2;

          SOC = CANEV.read();
          sniffs++;

          //If data file can't be opened, throw error.
          if (!dataFile) {
            Serial.println("Cannot save EV");
          }

          else {
            dataFile.print("EV");
            dataFile.print("\t");
            dataFile.print(millis());
            dataFile.print("\t");
            dataFile.print(ReadKmCar);
            dataFile.print("\t");
            dataFile.print("1db");
            dataFile.print("\t");
            dataFile.print(ReadHiEV, HEX);
            dataFile.print("\t");
            dataFile.print(ReadLoEV, HEX);
            dataFile.print("\t");
            dataFile.print(ReadCurrent);
            dataFile.print("\t");
            dataFile.print(ReadHiVoltsEV, HEX);
            dataFile.print("\t");
            dataFile.print(ReadLoVoltsEV, HEX);
            dataFile.print("\t");
            dataFile.print(ReadVolts);
            dataFile.print("\t");
            dataFile.println(SOC);
          }
        } else if (CANEV.packetId() == 0x1da) {
          MotorVoltage = (unsigned long)CANEV.read();
          MotorVoltage = MotorVoltage * 2;

          CANEV.read();

          ReadHiMotorEV = CANEV.read();
          ReadLoMotorEV = CANEV.read();
          ReadHiMotorEV = byte(ReadHiMotorEV & 0b00000111);
          ReadMotorTorque = short(((ReadHiMotorEV << 8) | ReadLoMotorEV) << 5);
          ReadMotorTorque2 = ReadMotorTorque / 32;
          ReadMotorTorque2 = ReadMotorTorque2 / 2;

          ReadHiMotorRPM = CANEV.read();
          ReadLoMotorRPM = CANEV.read();
          ReadMotorRPM = short((ReadHiMotorRPM << 8) | ReadLoMotorRPM);
          ReadMotorRPM = ReadMotorRPM / 2;

          MotorCurrentCalc = (ReadMotorTorque2 * ReadMotorRPM * 2 * pi) / (60 * ReadVolts);

          if (!dataFile) {
            
          }

          else {
            dataFile.print("EV");
            dataFile.print("\t");
            dataFile.print(millis());
            dataFile.print("\t");
            dataFile.print(ReadKmCar);
            dataFile.print("\t");
            dataFile.print("1da");
            dataFile.print("\t");
            dataFile.print(ReadHiMotorEV, HEX);
            dataFile.print("\t");
            dataFile.print(ReadLoMotorEV, HEX);
            dataFile.print("\t");
            dataFile.print(ReadMotorTorque2);
            dataFile.print("\t");
            dataFile.print(MotorVoltage);
            dataFile.print("\t");
            dataFile.print(ReadMotorRPM);
            dataFile.print("\t");
            dataFile.println(MotorCurrentCalc);
          }
        }
      } else {
      }
    }
    if (timeOut > (unsigned long)(millis() - startTime)) {
      CANCar.read();
      CANCar.read();
      CANCar.read();
      CANCar.read();
      ReadHiCar = CANCar.read();
      ReadLoCar = CANCar.read();
      ReadKmCar = ((ReadHiCar << 8) | ReadLoCar);
    }
  }

  digitalWrite(4, HIGH);
  CANCar.filter(0x70F);
  CANCar.beginPacket(0x70E, 8);
  CANCar.write(0x02);
  CANCar.write(0x21);
  CANCar.write(0x01);
  CANCar.endPacket();

  writeTimeOld = millis();
  startTime = millis();

  while ((CANCar.parsePacket() == 0 ||  //Waiting until reply comes from CarCAN
          CANCar.read() != 0x10 || CANCar.read() != 0x13 || CANCar.read() != 0x61 || CANCar.read() != 0x01)
         && timeOut > millis() - startTime) {  
    if (CANEV.parsePacket() > 0) {
      if (CANEV.packetId() == 0x1db) {
        ReadHiEV = CANEV.read();
        ReadLoEV = CANEV.read();
        ReadCurrent = float((ReadHiEV << 8) | ReadLoEV);
        ReadCurrent = float(ReadCurrent / 32);
        ReadCurrent = ReadCurrent / 2;

        ReadHiVoltsEV = CANEV.read();
        ReadLoVoltsEV = CANEV.read();
        ReadVolts = int((ReadHiVoltsEV << 8) | ReadLoVoltsEV);
        ReadVolts = (ReadVolts >> 6);
        ReadVolts = ReadVolts / 2;

        SOC = CANEV.read();
        sniffs++;

        //If data file can't be opened, throw error.
        if (!dataFile) {
          Serial.println("Cannot save EV");
        }

        else {
          dataFile.print("EV");
          dataFile.print("\t");
          dataFile.print(millis());
          dataFile.print("\t");
          dataFile.print(ReadKmCar);
          dataFile.print("\t");
          dataFile.print("1db");
          dataFile.print("\t");
          dataFile.print(ReadHiEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadLoEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadCurrent);
          dataFile.print("\t");
          dataFile.print(ReadHiVoltsEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadLoVoltsEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadVolts);
          dataFile.print("\t");
          dataFile.println(SOC);
        }
      } else if (CANEV.packetId() == 0x1da) {
        MotorVoltage = (unsigned long)CANEV.read();
        MotorVoltage = MotorVoltage * 2;

        CANEV.read();

        ReadHiMotorEV = CANEV.read();
        ReadLoMotorEV = CANEV.read();
        ReadHiMotorEV = byte(ReadHiMotorEV & 0b00000111);
        ReadMotorTorque = short(((ReadHiMotorEV << 8) | ReadLoMotorEV) << 5);
        ReadMotorTorque2 = ReadMotorTorque / 32;
        ReadMotorTorque2 = ReadMotorTorque2 / 2;

        ReadHiMotorRPM = CANEV.read();
        ReadLoMotorRPM = CANEV.read();
        ReadMotorRPM = short((ReadHiMotorRPM << 8) | ReadLoMotorRPM);
        ReadMotorRPM = ReadMotorRPM / 2;

        MotorCurrentCalc = (ReadMotorTorque2 * ReadMotorRPM * 2 * pi) / (60 * ReadVolts);

        if (!dataFile) {
        }

        else {
          dataFile.print("EV");
          dataFile.print("\t");
          dataFile.print(millis());
          dataFile.print("\t");
          dataFile.print(ReadKmCar);
          dataFile.print("\t");
          dataFile.print("1da");
          dataFile.print("\t");
          dataFile.print(ReadHiMotorEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadLoMotorEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadMotorTorque2);
          dataFile.print("\t");
          dataFile.print(MotorVoltage);
          dataFile.print("\t");
          dataFile.print(ReadMotorRPM);
          dataFile.print("\t");
          dataFile.println(MotorCurrentCalc);
        }
      }
    } else {
    }
  }
  if (timeOut > (unsigned long)(millis() - startTime)) {
    CANCar.beginPacket(0x70E, 8);
    CANCar.write(0x30);
    CANCar.endPacket();

    writeTimeOld = millis();
    startTime = millis();

    while ((CANCar.parsePacket() == 0 ||  //Waiting until reply comes from CarCAN
            CANCar.read() != 0x22 || CANCar.read() != 0x5E)
           && timeOut > millis() - startTime) {  
      if (CANEV.parsePacket() > 0) {
        if (CANEV.packetId() == 0x1db) {
          ReadHiEV = CANEV.read();
          ReadLoEV = CANEV.read();
          ReadCurrent = float((ReadHiEV << 8) | ReadLoEV);
          ReadCurrent = float(ReadCurrent / 32);
          ReadCurrent = ReadCurrent / 2;

          ReadHiVoltsEV = CANEV.read();
          ReadLoVoltsEV = CANEV.read();
          ReadVolts = int((ReadHiVoltsEV << 8) | ReadLoVoltsEV);
          ReadVolts = (ReadVolts >> 6);
          ReadVolts = ReadVolts / 2;

          SOC = CANEV.read();
          sniffs++;
          //If data file can't be opened, throw error.
          if (!dataFile) {
            Serial.println("Cannot save EV");
          }

          else {
            dataFile.print("EV");
            dataFile.print("\t");
            dataFile.print(millis());
            dataFile.print("\t");
            dataFile.print(ReadKmCar);
            dataFile.print("\t");
            dataFile.print("1db");
            dataFile.print("\t");
            dataFile.print(ReadHiEV, HEX);
            dataFile.print("\t");
            dataFile.print(ReadLoEV, HEX);
            dataFile.print("\t");
            dataFile.print(ReadCurrent);
            dataFile.print("\t");
            dataFile.print(ReadHiVoltsEV, HEX);
            dataFile.print("\t");
            dataFile.print(ReadLoVoltsEV, HEX);
            dataFile.print("\t");
            dataFile.print(ReadVolts);
            dataFile.print("\t");
            dataFile.println(SOC);
          }
        } else if (CANEV.packetId() == 0x1da) {
          MotorVoltage = (unsigned long)CANEV.read();
          MotorVoltage = MotorVoltage * 2;

          CANEV.read();

          ReadHiMotorEV = CANEV.read();
          ReadLoMotorEV = CANEV.read();
          ReadHiMotorEV = byte(ReadHiMotorEV & 0b00000111);
          ReadMotorTorque = short(((ReadHiMotorEV << 8) | ReadLoMotorEV) << 5);
          ReadMotorTorque2 = ReadMotorTorque / 32;
          ReadMotorTorque2 = ReadMotorTorque2 / 2;

          ReadHiMotorRPM = CANEV.read();
          ReadLoMotorRPM = CANEV.read();
          ReadMotorRPM = short((ReadHiMotorRPM << 8) | ReadLoMotorRPM);
          ReadMotorRPM = ReadMotorRPM / 2;

          MotorCurrentCalc = (ReadMotorTorque2 * ReadMotorRPM * 2 * pi) / (60 * ReadVolts);

          if (!dataFile) {
            
          }
          else {
            dataFile.print("EV");
            dataFile.print("\t");
            dataFile.print(millis());
            dataFile.print("\t");
            dataFile.print(ReadKmCar);
            dataFile.print("\t");
            dataFile.print("1da");
            dataFile.print("\t");
            dataFile.print(ReadHiMotorEV, HEX);
            dataFile.print("\t");
            dataFile.print(ReadLoMotorEV, HEX);
            dataFile.print("\t");
            dataFile.print(ReadMotorTorque2);
            dataFile.print("\t");
            dataFile.print(MotorVoltage);
            dataFile.print("\t");
            dataFile.print(ReadMotorRPM);
            dataFile.print("\t");
            dataFile.println(MotorCurrentCalc);
          }
        }
      } else {
      }
    }
    if (timeOut > (unsigned long)(millis() - startTime)) {
      ReadHiCar = CANCar.read();
      ReadLoCar = CANCar.read();
      ReadBrakePedalCar = ((ReadHiCar * 256) + ReadLoCar) * (1.3 / 256);
    }
  }

  CANCar.filter(0x760);
  CANCar.beginPacket(0x740, 8);
  CANCar.write(0x03);
  CANCar.write(0x22);
  CANCar.write(0x12);
  CANCar.write(0x09);
  CANCar.endPacket();
  writeTimeOld = millis();
  startTime = millis();

  while ((CANCar.parsePacket() == 0 ||  //Waiting until reply comes from CarCAN
          CANCar.read() < 5 || CANCar.read() != 0x62 || CANCar.read() != 0x12 || CANCar.read() != 0x09)
         && timeOut > millis() - startTime) {

    if (CANEV.parsePacket() > 0) {
      if (CANEV.packetId() == 0x1db) {
        ReadHiEV = CANEV.read();
        ReadLoEV = CANEV.read();
        ReadCurrent = float((ReadHiEV << 8) | ReadLoEV);
        ReadCurrent = float(ReadCurrent / 32);
        ReadCurrent = ReadCurrent / 2;

        ReadHiVoltsEV = CANEV.read();
        ReadLoVoltsEV = CANEV.read();
        ReadVolts = int((ReadHiVoltsEV << 8) | ReadLoVoltsEV);
        ReadVolts = (ReadVolts >> 6);
        ReadVolts = ReadVolts / 2;

        SOC = CANEV.read();
        sniffs++;
        //If data file can't be opened, throw error.
        if (!dataFile) {
          Serial.println("Cannot save EV");
        }

        else {
          dataFile.print("EV");
          dataFile.print("\t");
          dataFile.print(millis());
          dataFile.print("\t");
          dataFile.print(ReadKmCar);
          dataFile.print("\t");
          dataFile.print("1db");
          dataFile.print("\t");
          dataFile.print(ReadHiEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadLoEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadCurrent);
          dataFile.print("\t");
          dataFile.print(ReadHiVoltsEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadLoVoltsEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadVolts);
          dataFile.print("\t");
          dataFile.println(SOC);
        }
      } else if (CANEV.packetId() == 0x1da) {
        MotorVoltage = (unsigned long)CANEV.read();
        MotorVoltage = MotorVoltage * 2;

        CANEV.read();

        ReadHiMotorEV = CANEV.read();
        ReadLoMotorEV = CANEV.read();
        ReadHiMotorEV = byte(ReadHiMotorEV & 0b00000111);
        ReadMotorTorque = short(((ReadHiMotorEV << 8) | ReadLoMotorEV) << 5);
        ReadMotorTorque2 = ReadMotorTorque / 32;
        ReadMotorTorque2 = ReadMotorTorque2 / 2;

        ReadHiMotorRPM = CANEV.read();
        ReadLoMotorRPM = CANEV.read();
        ReadMotorRPM = short((ReadHiMotorRPM << 8) | ReadLoMotorRPM);
        ReadMotorRPM = ReadMotorRPM / 2;

        MotorCurrentCalc = (ReadMotorTorque2 * ReadMotorRPM * 2 * pi) / (60 * ReadVolts);

        if (!dataFile) {
        }

        else {
          dataFile.print("EV");
          dataFile.print("\t");
          dataFile.print(millis());
          dataFile.print("\t");
          dataFile.print(ReadKmCar);
          dataFile.print("\t");
          dataFile.print("1da");
          dataFile.print("\t");
          dataFile.print(ReadHiMotorEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadLoMotorEV, HEX);
          dataFile.print("\t");
          dataFile.print(ReadMotorTorque2);
          dataFile.print("\t");
          dataFile.print(MotorVoltage);
          dataFile.print("\t");
          dataFile.print(ReadMotorRPM);
          dataFile.print("\t");
          dataFile.println(MotorCurrentCalc);
        }
      }
    } else {
    }
  }

  if (timeOut > (unsigned long)(millis() - startTime)) {
    ReadHiCar = CANCar.read();
    ReadLoCar = CANCar.read();
    brakeValue = float((ReadHiCar << 8) | ReadLoCar);
    brakeValue = brakeValue / 1.536;

    digitalWrite(4, HIGH);
    CANCar.filter(0x79A);
    CANCar.beginPacket(0x797, 8);
    CANCar.write(0x03);
    CANCar.write(0x22);
    CANCar.write(0x11);
    CANCar.write(0x28);
    CANCar.endPacket();

    writeTimeOld = millis();
    startTime = millis();



    while ((CANCar.parsePacket() == 0 ||  //Waiting until reply comes from CarCAN
            CANCar.read() < 5 || CANCar.read() != 0x62 || CANCar.read() != 0x11 || CANCar.read() != 0x28)
           && timeOut > millis() - startTime) { 
      if (CANEV.parsePacket() > 0) {
        if (CANEV.packetId() == 0x1db) {
          ReadHiEV = CANEV.read();
          ReadLoEV = CANEV.read();
          ReadCurrent = float((ReadHiEV << 8) | ReadLoEV);
          ReadCurrent = float(ReadCurrent / 32);
          ReadCurrent = ReadCurrent / 2;

          ReadHiVoltsEV = CANEV.read();
          ReadLoVoltsEV = CANEV.read();
          ReadVolts = int((ReadHiVoltsEV << 8) | ReadLoVoltsEV);
          ReadVolts = (ReadVolts >> 6);
          ReadVolts = ReadVolts / 2;

          SOC = CANEV.read();
          sniffs++;

          //If data file can't be opened, throw error.
          if (!dataFile) {
            Serial.println("Cannot save EV");
          }

          else {
            dataFile.print("EV");
            dataFile.print("\t");
            dataFile.print(millis());
            dataFile.print("\t");
            dataFile.print(ReadKmCar);
            dataFile.print("\t");
            dataFile.print("1db");
            dataFile.print("\t");
            dataFile.print(ReadHiEV, HEX);
            dataFile.print("\t");
            dataFile.print(ReadLoEV, HEX);
            dataFile.print("\t");
            dataFile.print(ReadCurrent);
            dataFile.print("\t");
            dataFile.print(ReadHiVoltsEV, HEX);
            dataFile.print("\t");
            dataFile.print(ReadLoVoltsEV, HEX);
            dataFile.print("\t");
            dataFile.print(ReadVolts);
            dataFile.print("\t");
            dataFile.println(SOC);
          }
        } else if (CANEV.packetId() == 0x1da) {
          MotorVoltage = (unsigned long)CANEV.read();
          MotorVoltage = MotorVoltage * 2;

          CANEV.read();

          ReadHiMotorEV = CANEV.read();
          ReadLoMotorEV = CANEV.read();
          ReadHiMotorEV = byte(ReadHiMotorEV & 0b00000111);
          ReadMotorTorque = short(((ReadHiMotorEV << 8) | ReadLoMotorEV) << 5);
          ReadMotorTorque2 = ReadMotorTorque / 32;
          ReadMotorTorque2 = ReadMotorTorque2 / 2;

          ReadHiMotorRPM = CANEV.read();
          ReadLoMotorRPM = CANEV.read();
          ReadMotorRPM = short((ReadHiMotorRPM << 8) | ReadLoMotorRPM);
          ReadMotorRPM = ReadMotorRPM / 2;

          MotorCurrentCalc = (ReadMotorTorque2 * ReadMotorRPM * 2 * pi) / (60 * ReadVolts);

          if (!dataFile) {
            
          }

          else {
            dataFile.print("EV");
            dataFile.print("\t");
            dataFile.print(millis());
            dataFile.print("\t");
            dataFile.print(ReadKmCar);
            dataFile.print("\t");
            dataFile.print("1da");
            dataFile.print("\t");
            dataFile.print(ReadHiMotorEV, HEX);
            dataFile.print("\t");
            dataFile.print(ReadLoMotorEV, HEX);
            dataFile.print("\t");
            dataFile.print(ReadMotorTorque2);
            dataFile.print("\t");
            dataFile.print(MotorVoltage);
            dataFile.print("\t");
            dataFile.print(ReadMotorRPM);
            dataFile.print("\t");
            dataFile.println(MotorCurrentCalc);
          }
        }
      } else {
      }
    }
    if (timeOut > (unsigned long)(millis() - startTime)) {
      ReadHiCar = CANCar.read();
      ReadLoCar = CANCar.read();
      ReadNominalTorqueCar = (ReadHiCar << 8) | ReadLoCar;
      ReadNominalTorqueCar = ReadNominalTorqueCar / 64;
    }

    digitalWrite(4, HIGH);
    CANCar.filter(0x760);
    CANCar.beginPacket(0x740, 8);
    CANCar.write(0x03);
    CANCar.write(0x22);
    CANCar.write(0x11);
    CANCar.write(0x15);
    CANCar.endPacket();

    writeTimeOld = millis();
    startTime = millis();


    while ((CANCar.parsePacket() == 0 ||  //Waiting until reply comes from CarCAN
            CANCar.read() < 4 || CANCar.read() != 0x62 || CANCar.read() != 0x11 || CANCar.read() != 0x15)
           && timeOut > millis() - startTime) {

      if (CANEV.parsePacket() > 0) {
        if (CANEV.packetId() == 0x1db) {
          ReadHiEV = CANEV.read();
          ReadLoEV = CANEV.read();
          ReadCurrent = float((ReadHiEV << 8) | ReadLoEV);
          ReadCurrent = float(ReadCurrent / 32);
          ReadCurrent = ReadCurrent / 2;

          ReadHiVoltsEV = CANEV.read();
          ReadLoVoltsEV = CANEV.read();
          ReadVolts = int((ReadHiVoltsEV << 8) | ReadLoVoltsEV);
          ReadVolts = (ReadVolts >> 6);
          ReadVolts = ReadVolts / 2;

          SOC = CANEV.read();
          sniffs++;

          //If data file can't be opened, throw error.
          if (!dataFile) {
            Serial.println("Cannot save EV");
          }

          else {
            dataFile.print("EV");
            dataFile.print("\t");
            dataFile.print(millis());
            dataFile.print("\t");
            dataFile.print(ReadKmCar);
            dataFile.print("\t");
            dataFile.print("1db");
            dataFile.print("\t");
            dataFile.print(ReadHiEV, HEX);
            dataFile.print("\t");
            dataFile.print(ReadLoEV, HEX);
            dataFile.print("\t");
            dataFile.print(ReadCurrent);
            dataFile.print("\t");
            dataFile.print(ReadHiVoltsEV, HEX);
            dataFile.print("\t");
            dataFile.print(ReadLoVoltsEV, HEX);
            dataFile.print("\t");
            dataFile.print(ReadVolts);
            dataFile.print("\t");
            dataFile.println(SOC);
          }
        } else if (CANEV.packetId() == 0x1da) {
          MotorVoltage = (unsigned long)CANEV.read();
          MotorVoltage = MotorVoltage * 2;

          CANEV.read();

          ReadHiMotorEV = CANEV.read();
          ReadLoMotorEV = CANEV.read();
          ReadHiMotorEV = byte(ReadHiMotorEV & 0b00000111);
          ReadMotorTorque = short(((ReadHiMotorEV << 8) | ReadLoMotorEV) << 5);
          ReadMotorTorque2 = ReadMotorTorque / 32;
          ReadMotorTorque2 = ReadMotorTorque2 / 2;

          ReadHiMotorRPM = CANEV.read();
          ReadLoMotorRPM = CANEV.read();
          ReadMotorRPM = short((ReadHiMotorRPM << 8) | ReadLoMotorRPM);
          ReadMotorRPM = ReadMotorRPM / 2;

          MotorCurrentCalc = (ReadMotorTorque2 * ReadMotorRPM * 2 * pi) / (60 * ReadVolts);

          if (!dataFile) {

          }

          else {
            dataFile.print("EV");
            dataFile.print("\t");
            dataFile.print(millis());
            dataFile.print("\t");
            dataFile.print(ReadKmCar);
            dataFile.print("\t");
            dataFile.print("1da");
            dataFile.print("\t");
            dataFile.print(ReadHiMotorEV, HEX);
            dataFile.print("\t");
            dataFile.print(ReadLoMotorEV, HEX);
            dataFile.print("\t");
            dataFile.print(ReadMotorTorque2);
            dataFile.print("\t");
            dataFile.print(MotorVoltage);
            dataFile.print("\t");
            dataFile.print(ReadMotorRPM);
            dataFile.print("\t");
            dataFile.println(MotorCurrentCalc);
          }
        }
      } else {
      }
    }
    if (timeOut > (unsigned long)(millis() - startTime)) {
      accPos = CANCar.read();
      accPos = accPos / 2;
    }

    digitalWrite(4, HIGH);
    CANCar.filter(0x79A);
    CANCar.beginPacket(0x797, 8);
    CANCar.write(0x03);
    CANCar.write(0x22);
    CANCar.write(0x12);
    CANCar.write(0x54);
    CANCar.endPacket();

    writeTimeOld = millis();
    startTime = millis();

    while ((CANCar.parsePacket() == 0 ||  //Waiting until reply comes from CarCAN
            CANCar.read() < 5 || CANCar.read() != 0x62 || CANCar.read() != 0x12 || CANCar.read() != 0x54)
           && timeOut > millis() - startTime) {  

      if (CANEV.parsePacket() > 0) {
        if (CANEV.packetId() == 0x1db) {
          ReadHiEV = CANEV.read();
          ReadLoEV = CANEV.read();
          ReadCurrent = float((ReadHiEV << 8) | ReadLoEV);
          ReadCurrent = float(ReadCurrent / 32);
          ReadCurrent = ReadCurrent / 2;

          ReadHiVoltsEV = CANEV.read();
          ReadLoVoltsEV = CANEV.read();
          ReadVolts = int((ReadHiVoltsEV << 8) | ReadLoVoltsEV);
          ReadVolts = (ReadVolts >> 6);
          ReadVolts = ReadVolts / 2;

          SOC = CANEV.read();
          sniffs++;

          //If data file can't be opened, throw error.
          if (!dataFile) {
            Serial.println("Cannot save EV");
          }

          else {

            dataFile.print("EV");
            dataFile.print("\t");
            dataFile.print(millis());
            dataFile.print("\t");
            dataFile.print(ReadKmCar);
            dataFile.print("\t");
            dataFile.print("1db");
            dataFile.print("\t");
            dataFile.print(ReadHiEV, HEX);
            dataFile.print("\t");
            dataFile.print(ReadLoEV, HEX);
            dataFile.print("\t");
            dataFile.print(ReadCurrent);
            dataFile.print("\t");
            dataFile.print(ReadHiVoltsEV, HEX);
            dataFile.print("\t");
            dataFile.print(ReadLoVoltsEV, HEX);
            dataFile.print("\t");
            dataFile.print(ReadVolts);
            dataFile.print("\t");
            dataFile.println(SOC);
          }
        } else if (CANEV.packetId() == 0x1da) {
          MotorVoltage = (unsigned long)CANEV.read();
          MotorVoltage = MotorVoltage * 2;

          CANEV.read();

          ReadHiMotorEV = CANEV.read();
          ReadLoMotorEV = CANEV.read();
          ReadHiMotorEV = byte(ReadHiMotorEV & 0b00000111);
          ReadMotorTorque = short(((ReadHiMotorEV << 8) | ReadLoMotorEV) << 5);
          ReadMotorTorque2 = ReadMotorTorque / 32;
          ReadMotorTorque2 = ReadMotorTorque2 / 2;

          ReadHiMotorRPM = CANEV.read();
          ReadLoMotorRPM = CANEV.read();
          ReadMotorRPM = short((ReadHiMotorRPM << 8) | ReadLoMotorRPM);
          ReadMotorRPM = ReadMotorRPM / 2;

          MotorCurrentCalc = (ReadMotorTorque2 * ReadMotorRPM * 2 * pi) / (60 * ReadVolts);

          if (!dataFile) {
          }

          else {
            dataFile.print("EV");
            dataFile.print("\t");
            dataFile.print(millis());
            dataFile.print("\t");
            dataFile.print(ReadKmCar);
            dataFile.print("\t");
            dataFile.print("1da");
            dataFile.print("\t");
            dataFile.print(ReadHiMotorEV, HEX);
            dataFile.print("\t");
            dataFile.print(ReadLoMotorEV, HEX);
            dataFile.print("\t");
            dataFile.print(ReadMotorTorque2);
            dataFile.print("\t");
            dataFile.print(MotorVoltage);
            dataFile.print("\t");
            dataFile.print(ReadMotorRPM);
            dataFile.print("\t");
            dataFile.println(MotorCurrentCalc);
          }
        }
      } else {
      }
    }
    if (timeOut > (unsigned long)(millis() - startTime)) {
      ReadHiCar = CANCar.read();
      ReadLoCar = CANCar.read();
      ReadActualTorqueCar = (ReadHiCar << 8) | ReadLoCar;
      ReadActualTorqueCar = ReadActualTorqueCar / 64;
    }

    BatteryPower = ReadCurrent * ReadVolts;
    MotorPowerEV = ReadMotorTorque2 * (float(ReadMotorRPM) / 60) * 2 * pi;
    MotorPowerNominal = ReadNominalTorqueCar * (float(ReadMotorRPM) / 60) * 2 * pi;
    MotorPowerActual = ReadActualTorqueCar * (float(ReadMotorRPM) / 60) * 2 * pi;

    Serial.print(startTime);
    Serial.print(" Battery Current CAR: ");
    Serial.print(ReadCurrentCar);
    Serial.print(" Battery Current EV: ");
    Serial.print(ReadCurrent);
    Serial.print(" Battery Voltage EV: ");
    Serial.print(ReadVolts);
    Serial.print(" Inverter Voltage EV: ");
    Serial.print(MotorVoltage);
    Serial.print(" Motor Torque EV: ");
    Serial.print(ReadMotorTorque2);
    Serial.print(" Motor Amps CALCULATED: ");
    Serial.print(MotorCurrentCalc);
    Serial.print(" Motor RPM EV: ");
    Serial.print(ReadMotorRPM);
    Serial.print(" No. of sniffs: ");
    Serial.print(sniffs);
    Serial.print(" ");
    Serial.print(" Battery temperature: ");
    Serial.print(BatteryTemp);
    Serial.print(" Accelerator pedal position: ");
    Serial.print(accPos);
    Serial.print(" Brake pedal position: ");
    Serial.print(ReadBrakePedalCar);
    Serial.print(" brake pressure: ");
    Serial.print(brakeValue);
    Serial.print(" Mileage: ");
    Serial.print(ReadKmCar);
    Serial.print(" SOC: ");
    Serial.print(SOC);
    Serial.print(" Vehicle speed: ");
    Serial.print(ReadSpeedCar);
    Serial.print(" Nominal torque value: ");
    Serial.print(ReadNominalTorqueCar);
    Serial.print(" Actual engine torque value: ");
    Serial.print(ReadActualTorqueCar);
    Serial.print(" Battery power CALCULATED EVCAN: ");
    Serial.print(BatteryPower);
    Serial.print(" Motor power CALCULATED EVCAN: ");
    Serial.print(MotorPowerEV);
    Serial.print("  Motor power CALCULATED CAR NOMINAL: ");
    Serial.print(MotorPowerNominal);
    Serial.print(" Motor power CALCULATED CAR ACTUAL: ");
    Serial.println(MotorPowerActual);

    //If data file can't be opened, throw error.
    if (!dataFile) {
      Serial.println("Cannot save");
    }

    else {
      //Extra prints removed due to memory limitations on Arduino
      dataFile.print("CAR");
      dataFile.print("\t");
      dataFile.print(millis());
      dataFile.print("\t");
      //dataFile.print("Mileage: ");
      dataFile.print(ReadKmCar);
      dataFile.print("\t");
      //dataFile.print("Current: ");
      dataFile.print(ReadCurrentCar);
      dataFile.print("\t");
      //dataFile.print("Battery temperature: ");
      dataFile.print(BatteryTemp);
      dataFile.print("\t");
      //dataFile.print("Accelerator position: ");
      dataFile.print(accPos);
      dataFile.print("\t");
      //dataFile.print("Brake position: ");
      dataFile.print(ReadBrakePedalCar);
      dataFile.print("\t");
      //dataFile.print("brake pressure: ");
      dataFile.print(brakeValue);
      dataFile.print("\t");
      //dataFile.print("Vehicle speed: ");
      dataFile.print(ReadSpeedCar);
      dataFile.print("\t");
      //dataFile.print("Nominal torque: ");
      dataFile.print(ReadNominalTorqueCar);
      dataFile.print("\t");
      //dataFile.print("Actual torque: ");
      dataFile.print(ReadActualTorqueCar);
      dataFile.print("\t");
      //dataFile.print("Calculated battery power: ");
      dataFile.print(BatteryPower);
      dataFile.print("\t");
      //dataFile.print("Calculated motor power from EV CAN: ");
      dataFile.print(MotorPowerEV);
      dataFile.print("\t");
      //dataFile.print("Calculated motor power from nominal torque: ");
      dataFile.print(MotorPowerNominal);
      dataFile.print("\t");
      //dataFile.print("Calculated motor power from actual torque: ");
      dataFile.print(MotorPowerActual);
      dataFile.print("\t");
      //dataFile.print("No of sniffs: ");
      dataFile.println(sniffs);

      dataFile.flush();
      dataFile.close();  //Close data logging file
      digitalWrite(4, LOW);
    }

    currentEVSum = 0;
    sniffs = 0;
  }

  else {
    //This part is executed when system times out with no response from CAN buses
    Serial.print("startTime: ");
    Serial.print(startTime);
    Serial.print(" Previous battery Current: ");
    Serial.println(ReadCurrentCar);
  }
}