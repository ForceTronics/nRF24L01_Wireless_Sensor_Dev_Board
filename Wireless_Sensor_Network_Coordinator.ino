
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>

// start RF24 communication layer
RF24 radio(9,10);

// start RF24 network layer
RF24Network network(radio);

// Coordinator address
const uint16_t thisNode = 00;

// Structure of our payload coming from router and end devices
struct Payload
{
  float aDCTemp; //temperature from onboard sensor
  bool batState; //bool to communicate battery power level, true is good and false means battery needs to be replaced
};

void setup(void)
{
  Serial.begin(57600);
  Serial.println("Coordinator is online.....");
 
  SPI.begin();
  radio.begin();
  network.begin(90, thisNode);
}

void loop(void)
{
  //check network communication regularly
  network.update();

   RF24NetworkHeader header; //create header variable
   Payload payload; //create payload variable
  // Any data on the network ready to read
  while ( network.available() )
  {
    // If so, grab it and print it out
    network.read(header,&payload,sizeof(payload));
    Serial.print("The node this is from: ");
    Serial.println(header.from_node);
    Serial.print("Temperature: ");
    Serial.print(payload.aDCTemp);
    Serial.print(" Battery status: ");
    Serial.println(payload.batState);
  }
}
