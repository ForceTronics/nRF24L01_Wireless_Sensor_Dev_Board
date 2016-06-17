
#include "WSNode.h" //Wrapper for wireless sensor flex node design code
//#include <WSNode.h> //if you have WSNode in the Arduino library use this call
#include <RF24Network.h> //Library for networking nRF24L01s, using version https://github.com/TMRh20/RF24Network
#include <RF24.h> //Library for nRF24L01, using version https://github.com/TMRh20/RF24
#include <SPI.h> //nRF24L01 uses SPI communication
#include <EEPROM.h> //EEPROM functions
#include <Wire.h> //I2C library for STTS751 temp sensor

RF24 radio(9,10); //create object to control and communicate with nRF24L01
RF24Network network(radio); //Create object to use nRF24L01 in mesh network
WSNode wNode(network,radio); //create wireless node / sensor object

//This is an example struct for sending a payload of data with WSNode.
//Note that this is the same setup in the library, just using it as an example if you want to send a custom payload
struct payload_t
 {
    float tempSen; //temperature from onboard sensor
    bool batState; //bool to communicate battery power level, true is good and false means battery needs to be replaced
 };

void setup()
{
	SPI.begin(); //Start SPI communication
  wNode.nodeBegin(); //This must be called before network.begin
  radio.begin(); //start nRF24L01 communication and control
  //setup network communication, first argument is channel which determines frequency band module communicates on. Second argument is address of this module
  network.begin(90, wNode.thisNode); //Note that wNode.thisNode gets the node setting from EEPROM
  wNode.setTempToF();//set temp measurements for Cel
}

void loop()
{
  //This function sends built-in temp data and battery state data.
  //if in end device mode this function will sleep after it transmits, after sleeping it will wake and execute any other code in the loop
  //If in router mode this function will only transmit at the user set interval. When not transmitting the loop will continue to run
  if(!wNode.sendSensorData()) {
    //If this is true transmit failed
  }
  /*This commented out code show how to send a custom payload and get temperature reading as well as battery reading*/
  //payload_t payload = {wNode.getSTTS751Temp(), wNode.checkBatteryVolt()};
  //if(!wNode.sendCustomData(&payload, sizeof(payload))) {
    //if this is true transmit failed
//  }
//if you want to run serial monitor be sure to do Serial.end() to shut it off so it does not interfer with sleep settings
// Do not do any long delays in the loop or it will throw off the timing of the transmits, it can also cause problems with routers missing data they had to route from end device
 
}
