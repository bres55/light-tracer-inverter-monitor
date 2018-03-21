// 22/6/2017
/* Roof Transmitter
    Using UNO
  will get data from
  a  BH1750 Digital Light intensity sensor
  and a TEMT6000 Light Sensor
  and a Digital Temperature And Relative Humidity Sensor DHT11 Module
  It will transmit this info to the Downstairs_Receiver

   1 - GND
   2 - VCC 3.3V !!! NOT 5V,,  5V ok with adaptor
   3 - CE to Arduino pin 7
   4 - CSN to Arduino pin 8
   5 - SCK to Arduino pin 13
   6 - MOSI to Arduino pin 11
   7 - MISO to Arduino pin 12
   8 - UNUSED
*/
//#include "Arduino.h"
#include <SPI.h> // Comes with Arduino IDE
// #include <nRF24L01.h> ??
#include <RF24.h> // Download and Install...https://github.com/tmrh20/RF24/
#include <BH1750.h>
#include "DHT.h"
RF24 roofradio(7, 8); // my roof radio use pins, CE=7, CNS=8, on UNO and Mega

//https://www.reddit.com/r/arduino/comments/2qese9/is_my_understanding_of_uint8_t_addresses6/
//addresses[index_of_string][index_of_character]
//char *addresses[] = {"1Node", "2Node"}; // An array of strings
//char *addresses[] = {"RoofT", "DownR"}; // An array of strings
//Serial.println(addresses[0]); // prints "RoofT"
//Serial.println(addresses[1][0]); // prints "D" (from "DownR")
// addresses is as many as you like with []
//                       6 equals 5 chars long, strings are terminated by a null character,
// could be written,, byte addresses[2][6] = {"RoofT", "DownR"}; the compiler works it out there are 2

byte addresses[][6] = {"RoofT", "DownR"};
// address of bh1750
BH1750 lightMeter1(0x23);
BH1750 lightMeter2(0x5C);

// temp meter
// #define DHTPIN 5     // what digital pin we're connected to
// #define DHTTYPE DHT11
DHT dht(5, DHT11);
/*/http://www.instructables.com/id/Arduino-Wireless-Weather-Station/
  struct package
  {
  float temperature ;
  float humidity ;
  };
  typedef struct package Package;
  Package data;
  // end  1  http://www.instructables.com/id/Arduino-Wireless-Weather-Station/
*/
void setup() {
  Serial.begin(9600); // later
  // Initiate the radio object
  roofradio.begin();
  roofradio.setAutoAck(1); // Ensure autoACK is enabled
  // Set the speed of the transmission to moderate
  //roofradio.setDataRate(RF24_2MBPS);
  roofradio.setDataRate(RF24_250KBPS);

  // Use a channel unlikely to be used by Wifi
  roofradio.setChannel(124);

  // Set the transmit power to highest available
  // roofradio.setPALevel(RF24_PA_MIN);
  roofradio.setPALevel(RF24_PA_MAX);

  // Open a writing pipe
  roofradio.openWritingPipe(addresses[1]);
  delay(1000);
  // Open a reading pipe with opposite addresses
  // radio.openReadingPipe(1, addresses[0]);
  //
  lightMeter1.begin(BH1750_CONTINUOUS_HIGH_RES_MODE);
  lightMeter2.begin(BH1750_CONTINUOUS_LOW_RES_MODE);
  /* change to low res
 Full mode list:
      BH1750_CONTINUOUS_LOW_RES_MODE
      BH1750_CONTINUOUS_HIGH_RES_MODE (default)
      BH1750_CONTINUOUS_HIGH_RES_MODE_2
      BH1750_ONE_TIME_LOW_RES_MODE
      BH1750_ONE_TIME_HIGH_RES_MODE
      BH1750_ONE_TIME_HIGH_RES_MODE_2
      */
     
  //temperature sensor
  dht.begin();
}
void loop() {
  // read the lightmeter BH1750
  uint16_t   Lux1 = lightMeter1.readLightLevel();
  Serial.print(Lux1);
   Serial.println();
   uint16_t   Lux2 = lightMeter2.readLightLevel();
  Serial.print(Lux2);
  // read the temt6000
  int temt6000read = analogRead(A0);
  Serial.print(temt6000read);
  // Reading temperature or humidity on the dht11 takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  // Read temperature as Celsius (the default)
  uint16_t dht11read = dht.readTemperature();
  Serial.print(dht11read);

  // puts all 3 bits together with comma in between.
  //  int sprintf( char *buffer, const char *format, ... );
  //  buffer   -   pointer to a character string to write to
  //  format   -   pointer to a null-terminated multibyte string specifying how to interpret the data.
  //  The format string consists of ordinary multibyte characters (except %),
  //  which are copied unchanged into the output stream, and conversion specifications.
  //  Each conversion specification has the following format:
  //        introductory % character
  // d converts a signed integer into decimal representation [-]dddd. 
  // u converts an unsigned integer into decimal representation dddd. 
  // http://en.cppreference.com/w/c/io/fprintf
  //sprintf(text, "%d,%d,%d,%d", Lux1,Lux2, temt6000read, dht11read);
  char text[36];
  sprintf(text, "%u,%u,%u,%d", Lux1, Lux2, temt6000read, dht11read);// only temp is signed, wont know till a very cold night
  Serial.println(text);
 Serial.println();
  // Ensure we have stopped listening (even if we're not) or we won't be able to transmit
  roofradio.stopListening();
  // get time
  unsigned long started_waiting_at = millis();
  // Did we manage to SUCCESSFULLY transmit that,
  // setAutoAck set on by default
  // (by getting an acknowledgement back from the other Arduino)?

  // wait 5 seconds, if no response from receiver, go back get new data
  // dont want to be transmitting old data

  /* while (!roofradio.write(&text, sizeof(text) )) {
     if (millis() - started_waiting_at > 200 ) {
       Serial.println("No response received - timeout!");
       return;// loop starts again
     }
    }*/
  roofradio.write(&text, sizeof(text));

  // try this
  // roofradio.flush_tx() ;
  //const char text2[] = "Hello Michael World from a distance";
  //  Serial.println(text2);
  //delay(5000); //above made it work, so try delay ??? hand shaking??

  //By using the “&” before the variable name we actually set an indication of the variable that stores the data
  //that we want to be sent and using the second argument we set the number of bytes that we want to take from that variable.
  // In this case the sizeof() function gets all bytes of the strings “text”. At the end of the program we will add 1 second delay.


  delay(1000);//????????
}

