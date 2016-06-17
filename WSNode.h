#pragma once
/*
WSNode.h - Library for nRF24L01 wireless flex node
This library was createtd for the wireless sensor development board found on ForceTronics.com
A video series on the board and code can be found on the ForceTronics YouTube channel
Created by ForceTronics LLC 6 13 16
Released into the public domain.
*/
#ifndef WSNode_h
#define WSNode_h

#include "Arduino.h"
#include <RF24Network.h> //Library for networking nRF24L01s, using version https://github.com/TMRh20/RF24Network
#include <RF24.h>   //Library for networking nRF24L01s, using version https://github.com/TMRh20/RF24

class WSNode
{
public:
  //WSNode constructor, pulls in reference from RF24Network object and RF24 object from TMR libraries
   WSNode(RF24Network& _network, RF24& _radio):network(_network), radio(_radio){}
	~WSNode();

  //This function sets initial condition of board and gets setting from EEPROM
  //this should be called in "setup" Arduino setup function
  void nodeBegin();

  //this function sends temp sensor data and battery state to coordinator
  //If in router mode the function will use a timer to send at the set interval
  //if in end device mode this function will send data and go to sleep for specified time interval
  bool sendSensorData();

  //this function sends data from router or end device to the coordinator
  //input parameter can be struct with data you want to send
  //If in router mode the function will use a timer to send at the set interval
  //if in end device mode this function will send data and go to sleep for specified time interval
  bool sendCustomData(const void* message, uint16_t len);

  //Allows user to change settings and stores them in EEPROM. The settings will then be used from EEPROM next time the node is powered up
  //To change settings again press reset and hold down other button with serial monitor open
  void getSettings();

  //This function makes 8 ADC measurements but does nothing with them
  //Since after a reference change the ADC can return bad readings. This function is used to get rid of the first 
  //8 readings to ensure next reading is accurate
  void burn8Readings(int pin);

  //This function converts the ADC level integer value into float voltage value.
  //The inputs are the measured ADC value and the ADC reference voltage level
  //The formula used was obtained from the data sheet: (ADC value / 1024) x ref voltage
  float convertToVolt(float refVal, int aVAL) { return (((float)aVAL / 1024)*refVal); }

  //This function gets the two bytes of temperature data from the STTS751 Sensor and convert float temp value
  //This function was leveraged from blog post http://mike.saunby.net/2013_03_01_archive.html
  float getSTTS751Temp();

  //This function converts a C temp reading to a F reading
  float convertCtoF(float cValue) { return cValue*1.8 + 32; }

  //used to average multiple ADC values together. This can help eliminate noise in measurements
  int averageADCReadings(int aDCpin, int avgCount);

  //This function uses the known internal reference value of the 328p (~1.1V) to calculate the VCC value which comes from a battery
  //This was leveraged from a great tutorial found at https://code.google.com/p/tinkerit/wiki/SecretVoltmeter?pageId=110412607001051797704
  float fReadVcc();

  //This function checks the battery voltage and let's the coordinator know when the battery is getting low
  //If the battery gets too low this will trigger a constant sleep state
  bool checkBatteryVolt();

  void setTempToF(); //sets temp to F

  void setTempToC(); //sets temp to C

  void enableBatShutdown(); //does battery check when in end device mode
  void disableBatShutdown(); //does battery check when in end device mode

   //Variables
  RF24Network& network; //reference variable for RF24Network object
  RF24& radio; //reference varaible for RF24 object
  int thisNode = 1; //address of this router or end device
  float iREF = 1.1; //Voltage value of the internal 1.1V reference, measure actual and set it here for better measurement accuracy
  float lowBat = 2.6; //voltage value for low battery
  float deadBat = 2.4; //voltage value for dead battery
  bool batShutdown = false; //tracks if battery is good or dead
  unsigned char mMode; //used to track whether in end device mode or router mode
  //structure used to hold the payload that is sent to the coordinator. 
  //Currently setup to send temp value as float, ADC value as int, and battery state as bool
  //Can be adjusted to accomodate the exact data you want node to send
  //Keep in mind the more data you send the more power is used and the more likely
  //there is some kind of data error in the transmit receive process
  struct payload_t
  {
    float aDCTemp; //temperature from onboard sensor
    bool batState; //bool to communicate battery power level, true is good and false means battery needs to be replaced
  };

  private:
  //timer function that let's the node know when to transmit in router mode
  //function returns true when it is time to transmit
  bool checkTimer();

  //This function sets the sleep interval timer
  //Choices are 0 = 1 sec, 1 = 1 min (4 sec sleep interval 15), 2 = 10 min (8 sec sleep interval 75), 3 = 15 min (4 sec interval 225), 4 = 60 min (8 sec interval 450)
  void setSleepInterval(byte interval);

  //This function communicates with the STTS751 over I2C and gets a byte of temperature data
  //inputs are device address and the address of the byte to read
  //This function was leveraged from blog post http://mike.saunby.net/2013_03_01_archive.html
  unsigned char i2c_sensor_read_byte(int deviceaddress, int eeaddress);

  //This function serves as a power saving delay function. The argument is a Byte type variable that is used to set the delay time
  //The function sets up sleep mode in power down state. The function then sets up the WDT timer in interrupt mode.
  //It then puts the Arduino to sleep for the set time. Upon wake up the WDT and sleep mode are shut off
  bool goToSleep();

  //This function gets stored settings from EEPROM and stores in global variables
  //The addresses are hard coded and are spread out based on size of stored variable
  //each address is a byte in length
  void getEEPROMValues();

  //Checks if user entered a 'Y' into the serial monitor if so returns true
  bool getAnswer();

  //gets transmit interval setting from EEPROM and prints it out to user
  char* getInterval();

  //gets mode setting from EEPROM and prints it out to user
  char* getMode();
  
  bool tempSetF = true; //if true temperature will be given in F, if false it will be given in C
  bool batCheckOn = true; //if true the code will check if battery is good when in end device mode, if false will not check battery when in end device mode
  bool timer = true; //Used while in router mode to set transmit interval
  byte sInterval; //used to set interval between transmitting data
  unsigned long tInterval; //Used for transmit time interval in router mode
  unsigned long tStart; //Used for transmit time interval in router mode
  const unsigned short int rXNode = 00; //address of coordinator
  byte wDTInterval; //used for tracking sleep interval
  byte count; //used for tracking sleep interval
};

#endif
