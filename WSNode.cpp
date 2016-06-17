/*
WSNode.h - Library for nRF24L01 wireless flex node
This library was createtd for the wireless sensor development board found on ForceTronics.com
A video series on the board and code can be found on the ForceTronics YouTube channel
Created by ForceTronics LLC 6 13 16
Released into the public domain.
*/

#include "WSNode.h"
#include "Arduino.h"
#include <EEPROM.h> //EEPROM functions
#include <Wire.h> //I2C library for STTS751 temp sensor
#include <RF24Network.h> //Library for networking nRF24L01s, using version https://github.com/TMRh20/RF24Network
#include <avr/sleep.h> //Sleep functions
#include <avr/wdt.h> //Watch dog timer functions


WSNode::~WSNode()
{
}

//This function sets initial condition of board and gets setting from EEPROM
//this should be called in "setup" Arduino setup function
void WSNode::nodeBegin() {
  analogReference(INTERNAL); //set the ADC reference to internal 1.1V reference
  burn8Readings(A0); //make 8 readings but don't use them to ensure good reading after ADC reference change
  pinMode(7, OUTPUT); //set digital pin 7 as output to control status LED
  pinMode(5,OUTPUT); //set to output, connected to interrupt pin of STTS751 temp sensor
  digitalWrite(5,HIGH); //Need this to pull interrupt pin of STTS751 high (it cannot float)
  getSettings(); //Function used for updating and getting module settings from EEPROM
  setSleepInterval(sInterval);   //Used to set interval between transmits, based on settings data from EEPROM
}


//this function sends temp sensor data and battery state to coordinator
//If in router mode the function will use a timer to send at the set interval
//if in end device mode this function will send data and go to sleep for specified time interval
bool WSNode::sendSensorData() {
   bool xMit = true; //this stay true if transmit was good
  if(!batShutdown| !batCheckOn) { //check to see if we are in battery shutdown, if so all the module will do is sleep
    network.update(); //check to see if there is any network traffic that needs to be passed on, technically an end device does not need this 
    
    if(timer) { //If in router mode this used to track when it is time to transmit again
      //create packet to transmit. First argument calculates temperature, second gets ADC reading from A2, third gives state of battery
      payload_t payload = { getSTTS751Temp(), checkBatteryVolt()};
      RF24NetworkHeader header(rXNode); //Create transmit header. This goes in transmit packet to help route it where it needs to go, in this case it is the coordinator
      //send data onto network and make sure it gets there1
      if (network.write(header,&payload,sizeof(payload))) {
        digitalWrite(7,LOW); //transmit was successful so make sure status LED is off
      }
      else  { //transmit failed, try again
        if (!network.write(header,&payload,sizeof(payload))) {
          PIND |= (1<<PIND7); //this toggles the status LED at pin seven to show transmit failed
          xMit = false; //transmit failed so let the user know
        }
        else  digitalWrite(7,LOW); //transmit was successful so make sure status LED is off
      }
    }
  }
  
 if(!mMode) { //if we are in end device mode get ready to go to sleep
    timer = true; //wer are not using the timer so make sure variable is set to true
    radio.powerDown(); //power down the nRF24L01 before we go to sleep
    ADCSRA &= ~(1<<ADEN); //Turn off ADC before going to sleep (set ADEN bit to 0). this saves even more power
    goToSleep(); //function for putting the Atmega328 to sleep
    ADCSRA |= (1<<ADEN); //Turn the ADC back on
    radio.powerUp();
  }
  else { //we are in router mode 
    timer = checkTimer(); //update timer and check to see if it is time to transmit
    batShutdown = false; //this should never be true when in router mode
  }
  return xMit; //let user know if transmit was successful
}

//this function sends data from router or end device to the coordinator
//input parameter can be struct with data you want to send
//If in router mode the function will use a timer to send at the set interval
//if in end device mode this function will send data and go to sleep for specified time interval
bool WSNode::sendCustomData(const void* message, uint16_t len) {
   bool xMit = true; //this stay true if transmit was good
  if(!batShutdown | !batCheckOn) { //check to see if we are in battery shutdown, if so all the module will do is sleep
    network.update(); //check to see if there is any network traffic that needs to be passed on, technically an end device does not need this 
    
    if(timer) { //If in router mode this used to track when it is time to transmit again
      RF24NetworkHeader header(rXNode); //Create transmit header. This goes in transmit packet to help route it where it needs to go, in this case it is the coordinator
      //send data onto network and make sure it gets there
      //payload_t payload = { getSTTS751Temp(), checkBatteryVolt()};
      if (network.write(header,message,len)) {
        digitalWrite(7,LOW); //transmit was successful so make sure status LED is off
      }
      else  { //transmit failed, try again
        if (!network.write(header,message,len)) {
          PIND |= (1<<PIND7); //this toggles the status LED at pin seven to show transmit failed
          xMit = false; //transmit failed so let the user know
        }
        else  digitalWrite(7,LOW); //transmit was successful so make sure status LED is off
      }
    }
  }
  
 if(!mMode) { //if we are in end device mode get ready to go to sleep
    timer = true; //wer are not using the timer so make sure variable is set to true
    radio.powerDown(); //power down the nRF24L01 before we go to sleep
    ADCSRA &= ~(1<<ADEN); //Turn off ADC before going to sleep (set ADEN bit to 0). this saves even more power
    goToSleep(); //function for putting the Atmega328 to sleep
    ADCSRA |= (1<<ADEN); //Turn the ADC back on
    radio.powerUp();
  }
  else { //we are in router mode 
    timer = checkTimer(); //update timer and check to see if it is time to transmit
    batShutdown = false; //this should never be true when in router mode
  }
  return xMit; //let user know if transmit was successful
}

//timer function that let's the node know when to transmit in router mode
//function returns true when it is time to transmit
//This allows the main loop to keep executing when it is not time to transmit so network.update and be called cont
bool WSNode::checkTimer()
{
	unsigned long now = millis(); //get timer value
  if ( now - tStart >= tInterval  ) //check to see if it is time to transmit based on set interval
  {
    tStart = now; //reset start time of timer
    return true;
  }
  else return false;
}

//This function sets the sleep interval timer
//network.setup_watchdog choices are 0 = 1 sec, 1 = 1 min (4 sec sleep interval 15), 2 = 10 min (8 sec sleep interval 75), 3 = 15 min (4 sec interval 225), 4 = 60 min (8 sec interval 450)
void WSNode::setSleepInterval(byte interval) {

 //Following code sets the two variables for establishing the sleep interval on the ATmega328
  // sets the watch dog timer sleep interval at either 1 sec, 4 sec, 8 sec
  //count sets how many times we should loop the sleep cycle to equal the user defined transmit interval (1 sec, 1 min, 10 min, 15 min)
  if(interval == 0) { //set transmit interval to 1 sec
    count = 1;
    tInterval = 1000; //sets interval for timer when in router mode
    network.setup_watchdog(6); //Sets sleep interval of WDT, wdt_1s
  }
  else if(interval == 1) { //set transmit interval to 1 min
    count = 15;
    tInterval = 60000;
    network.setup_watchdog(8); //Sets sleep interval of WDT, wdt_4s
  }
  else if(interval == 2) { //sets transmit interval to 10 min
    count = 75;
    tInterval = 600000;
    network.setup_watchdog(9); //Sets sleep interval of WDT, wdt_8s
  }
  else {
    count = 225;
    tInterval = 900000;
    network.setup_watchdog(8); //Sets sleep interval of WDT, wdt_4s
  }
}

//This function serves as a power saving delay function. The argument is a Byte type variable that is used to set the delay time
//The function sets up sleep mode in power down state. The function then sets up the WDT timer in interrupt mode.
//It then puts the Arduino to sleep for the set time. Upon wake up the WDT and sleep mode are shut off
bool WSNode::goToSleep() {
  if(network.sleepNode(count,255)) return true; //count sets the number of intervals to sleep, 255 means do not wake up on interrupt
  else return false;
}

//This functions allows the user to set some of the main settings via the serial monitor
//serial monitor setup menu will execute the first time you start the module. After that settings are remembered in EEPROM next time you power module
//To adjust settings again reset the module and hold down the non-reset toggle switch to enter setting menu
//When just using settings from EEPROM this function just gets the setting from EEPROM and stores them in global variables
void WSNode::getSettings()
{
	pinMode(8, INPUT_PULLUP); //set pin 8 as an input
	int val; //variable to store node address
	byte cAddr = 128; //this variable is used to see if this is the first time the module is being used
	byte tInterval; //set the transmit time interval
	byte mode; //set to router or end device mode
			   //The following strings are used by serial monitor more than once so to save memory they are made into variables
	String enter = F("Enter 'Y' for yes or any other character for no:"); //F() macro tells IDE to store string in flash memory and not SRAM
	String invalid = F("Invalid entry, default to ");
	String would = F("Would you like to update the ");

	//if this is either the first time the module is used or if pin 8 is connecting to ground enter settings mode
	if (EEPROM.get(0, cAddr) != 128 || !digitalRead(8)) {
		digitalWrite(7, HIGH); //in settings mode so turn on status LED
		Serial.begin(57600); //start serial communication, need to turn off before using sleep and WDT
							 //the following code reads the current settings from EEPROM and puts them in local variables
		Serial.println(F("Current settings in EEPROM"));
		Serial.print("Node address: ");
		Serial.println(EEPROM.get(1, val), OCT);
		Serial.print("Time interval: ");
		Serial.println(getInterval());
		Serial.print("Mode: ");
		Serial.println(getMode());
		//This following gives you the option to set each setting and if you change a setting it is stored in EEPROM
		Serial.print(would);
		Serial.print("Node Address? ");
		Serial.println(enter);
		if (getAnswer()) {
			Serial.println(F("Enter Node address to store in EEPROM"));
			while (!Serial.available()) {}
			val = Serial.parseInt();
			if (val >= 0) {
				EEPROM.put(1, val);
			}
			else { //if zero is entered it is invalid since coordinator is zero
				Serial.print(invalid);
				Serial.println("01");
				val = 01;
				EEPROM.put(1, val);
			}
		}

		Serial.print(would);
		Serial.print("time interval? ");
		Serial.println(enter);
		if (getAnswer()) {
			Serial.println(F("Enter: 0 for 1 sec, 1 for 1 min, 2 for 10 min, 3 for 15 min"));
			while (!Serial.available()) {}
			tInterval = Serial.parseInt();
			if (tInterval >= 0 || tInterval < 4) { //check that entry was valid
				EEPROM.put(7, tInterval);
			}
			else {
				Serial.print(invalid);
				Serial.println("3");
				tInterval = 3;
				EEPROM.put(7, tInterval);
			}
		}
		Serial.print(would);
		Serial.print("mode setting? ");
		Serial.println(enter);
		if (getAnswer()) {
			Serial.println(F("Enter 0 for end device and 1 for router"));
			while (!Serial.available()) {}
			mode = Serial.parseInt();
			if (mode == 0 || mode == 1) { //check that entry was valid
				EEPROM.put(8, mode);
			}
			else {
				Serial.print(invalid);
				Serial.println("0");
				mode = 0;
				EEPROM.put(8, mode);
			}
     }
    getEEPROMValues(); //gets settings from EEPROM and stor in global variables
    //the following code prints out current settings from global variables
    Serial.print("Node address: ");
    Serial.println(thisNode,OCT);
    Serial.print("Time interval: ");
    Serial.println(sInterval);
    Serial.print("Mode setting: ");
    Serial.println(mMode);
    cAddr = 128; //write '128' to EEPROM to show that settings have been entered at least once
    EEPROM.put(0, cAddr);
    Serial.end(); //need to end serial communication before using the WDT and sleep functions
  }
  else {
     //not in settings mode so just get settings from EEPROM and store in global variables
    getEEPROMValues();
  }
}

//Gets setting values from EEPROM and add them to appr global variables
void WSNode::getEEPROMValues()
{
  EEPROM.get(1,thisNode);
  EEPROM.get(7,sInterval);
  EEPROM.get(8,mMode);
}


//gets reponse from serial monitor to see if user wants to change particular setting
bool WSNode::getAnswer()
{
	 while (!Serial.available()) { }
   if(Serial.read() == 'Y') return true;
   else return false;
}

//returns string that says what transmit interval we are in
//checks transmit interval by accessing mode variable from EEPROM
char * WSNode::getInterval()
{
	byte i;
  EEPROM.get(7,i);
  if(i==0) {
    return "1sec";
  }
  else if (i==1) {
    return ("1min");
  }
  else if (i==2) {
    return ("10min");
  }
  else {
     EEPROM.put(7,i);
    return ("15min");
  }
}

//returns string that says what mode we are in
//checks mode by accessing mode variable from EEPROM
char * WSNode::getMode()
{
	 byte m;
  EEPROM.get(8,m);
  if(m==1) {
    return "Router";
  }
  else {
    m = 0;
    EEPROM.put(8,m);
    return "End Device";
  }
}

//This function makes 8 ADC measurements but does nothing with them
//Since after a reference change the ADC can return bad readings. This function is used to get rid of the first 
//8 readings to ensure next reading is accurate
void WSNode::burn8Readings(int pin)
{
  for(int i=0; i<8; i++) {
    analogRead(pin);
  }
}

//This function gets the two bytes of temperature data from the STTS751 Sensor and convert float temp value
//This function was leveraged from blog post http://mike.saunby.net/2013_03_01_archive.html
float WSNode::getSTTS751Temp()
{
	byte lo;
  signed char hi;
//Because the Address pin is tied to ground the STTS751 address is 0x3B
  Wire.begin(); // initialise the connection
  hi = i2c_sensor_read_byte(0x3B, 0);
  lo = i2c_sensor_read_byte(0x3B, 2);
  Wire.end(); //turn wire off until next reading
  if(tempSetF) {
    if( hi > 0) return convertCtoF(hi + lo * 1.0/256.0);
    else return convertCtoF(hi - lo * 1.0/256.0); 
  }
  else {
    if( hi > 0) return (hi + lo * 1.0/256.0);
    else return (hi - lo * 1.0/256.0); 
  }
}

//This function communicates with the STTS751 over I2C and gets a byte of temperature data
//inputs are device address and the address of the byte to read
//This function was leveraged from blog post http://mike.saunby.net/2013_03_01_archive.html
unsigned char WSNode::i2c_sensor_read_byte(int deviceaddress, int eeaddress)
{
	byte rdata = 0xFF;
  int rc;
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)eeaddress);
  rc = Wire.endTransmission();
  Wire.requestFrom(deviceaddress,1);
  if (Wire.available()){
    rdata = Wire.read();
  }
  if(rc != 0){
    Serial.print("Error ");
    Serial.println(rc);
  } 
  return rdata;
}

//used to average multiple ADC values together
//This can help eliminate noise in measurements
int WSNode::averageADCReadings(int aDCpin, int avgCount)
{
	int aDCAvg = 0;
 for(int i=0;i<avgCount;i++) {
  aDCAvg = aDCAvg + analogRead(aDCpin);
 }

 return (aDCAvg/avgCount);
}

//This function uses the known internal reference value of the 328p (~1.1V) to calculate the VCC value which comes from a battery
//This was leveraged from a great tutorial found at https://code.google.com/p/tinkerit/wiki/SecretVoltmeter?pageId=110412607001051797704
float WSNode::fReadVcc()
{
	analogReference(EXTERNAL); //set the ADC reference to AVCC 
  burn8Readings(A0); //make 8 readings but don't use them to ensure good reading after ADC reference change 
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  unsigned long start = millis(); //get timer value
  while ( (start + 3) > millis()); //delay for 3 milliseconds
  ADCSRA |= _BV(ADSC); // Start ADC conversion
  while (bit_is_set(ADCSRA,ADSC)); //wait until conversion is complete
  int result = ADCL; //get first half of result
  result |= ADCH<<8; //get rest of the result
  float batVolt = (iREF / result)*1024; //Use the known iRef to calculate battery voltage
  analogReference(INTERNAL); //set the ADC reference back to internal
  burn8Readings(A0); //make 8 readings but don't use them to ensure good reading after ADC reference change 
  return batVolt;
}

//This function checks the battery voltage and let's the coordinator know when the battery is getting low
//If the battery gets too low this will trigger a constant sleep state
bool WSNode::checkBatteryVolt()
{
  //Get ADC reading. Converter to a voltage. Multiple by battery cal factor to get true battery voltage
  //float batVal = convertToVolt(iREF,averageADCReadings(aPin,20))*bVoltDivide;
  float batVal = fReadVcc(); //get battery voltage by measuring internal ref with AVCC 

  if(batVal < deadBat) { 
    batShutdown = true; //battery is 'dead' the set battery shut down variable
    return false;
  }
  else if(batVal < lowBat) return false; //This will alert the coordinator that the battery power is low
  else return true; //battery is fine 
}

void WSNode::setTempToF() { //sets temp to F
   tempSetF = true;
}

void WSNode::setTempToC() { //sets temp to C
  tempSetF = false;
}

void WSNode::enableBatShutdown() { //This enables battery shutdown mode
  batCheckOn = true;
}

void WSNode::disableBatShutdown() { //This disables battery shutdown mode
  batCheckOn = false;
}
