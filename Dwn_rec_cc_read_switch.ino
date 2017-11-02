// 9/10/2017
/* Downstairs_Receiver
    Using Mega 256
  will get data from
  Roof Transmitter
  1 - GND
   2 - VCC 3.3V !!! NOT 5V
   3 - CE to Arduino pin 7
   4 - CSN to Arduino pin 8
   5 - SCK to Arduino pin 13
   6 - MOSI to Arduino pin 11
   7 - MISO to Arduino pin 12
   8 - UNUSED
  And before that, charge controller and SDM220 Modbus meter
*/

#include <SPI.h> // Comes with Arduino IDE
#include <RF24.h> // Download and Install...https://github.com/tmrh20/RF24/

#include <Bounce2.h> //https://github.com/thomasfredericks/Bounce2/wiki
const int switchpin = 2;    // the number of the pushbutton pin
const int ledPin = 13;      // the number of the LED pin to show Oad on/off
// Instantiate a Bounce object :
Bounce debouncer = Bounce();

// CONNECT THE RS485 MODULE RX->RX, TX->TX.
// Disconnect when uploading code.
#include <ModbusMaster.h> // https://github.com/4-20ma/ModbusMaster

float PanelsV, PanelsC, PanelsW, BatteryV, BatteryCC, LoadW, BatteryTemp, MaxPanelVToday, MaxBatVToday;
float MinBatVToday, GeneratedEnergyToday, HVD, CLV, OVR, EQUV, BoostV, FloatV, BoostRecV, LVR, LVD;
word ChrgStatus;//1=on 0=off,,,,D3-D2 00=No charge, 01/1=Float, 10/2=Boost, 11/3=Equalisation
bool LoadOn;
// float MtrV, MtrC, MtrW, MtrKWh1, MtrKWh2,MtrKWh3, MtrKWh;
uint8_t result;

// initialise ModbusMaster object
//better names
ModbusMaster Tracer_modbus; //old node
ModbusMaster sdm220_modbus; // old node2
// What we would do pre and post transmission. nothing
// tracer requires no handshaking
void preTransmission() {}
void postTransmission() {}

// this is to check if we can write since rs485 is half duplex ??? not sure what this is about??
bool rs485DataReceived = true;

RF24 downradio(7, 8); // my down radio uses pins on UNO, and receiver is a Mega CE=7, CNS=8
// Only RoofT used as I dont send any info back yet
byte addresses[][6] = {"RoofT", "DownR"};

// UNION, used to re-jig float
// used to get numbers from meter
union ifloat {
  uint8_t bytes[4];
  float val;
};
// END UNION
//ssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssss
void setup()
{
  Serial.begin(9600); // will be sending all data to serial, for later analysis
  // Initiate the radio object
  downradio.begin();
  // Set the speed of the transmission to moderate
  //roofradio.setDataRate(RF24_2MBPS);
  downradio.setDataRate(RF24_250KBPS);
  // Use a channel unlikely to be used by Wifi
  downradio.setChannel(124);
  // Set the transmit power to highest available
  // downradio.setPALevel(RF24_PA_MIN);
  downradio.setPALevel(RF24_PA_MAX);
  // Open a reading pipe,  using the radio.setReadingPipe() function we set the same address
  // in that way we enable the communication between the two modules.
  downradio.openReadingPipe(1, addresses[1]);
  // Now listen for a response
  downradio.startListening();

  //  Initialize Modbus communication baud rate, usually 115200
  // Tracer--- Charge Controller
  // On a Mega 256, using Serial 1, Modbus slave ID 2.. my test tracer is on 2, usually 1;
  Serial1.begin(115200);
  Tracer_modbus.begin(1, Serial1);//1 tracer
  // SDM220 meter
  // On a Mega 256, using Serial 2, Modbus slave ID 1
  Serial2.begin(2400);
  sdm220_modbus.begin(1, Serial2);//1 sdm220
  //let it --Tracer_modbus-- know what we want for pre and post transmission, we call above void,
  //which was--do nothing.
  Tracer_modbus.preTransmission(preTransmission);
  Tracer_modbus.postTransmission(postTransmission);
  sdm220_modbus.preTransmission(preTransmission);
  sdm220_modbus.postTransmission(postTransmission);

  // to excel step one---------not used at mo.
  //Serial.println("CLEARDATA"); // clears starting at row 2
  // Serial.println("CLEARSHEET"); // clears starting at row 1
  //Serial.println("LABEL,Date,Time,PanV,PanC,PanP,BatV,BatC,LdP,BatTp,MaxP,MaxB,MinB,PKwh,HVD,CLV,OVR,EQUV,BSTV,FLTV,BRV,LVR,LVD,LdOn,Csts,MtrV,MtrC,MtrW,MtrKwh");

  // for load on off switch
  //configure switchpin2 as an input and enable the internal pull-up resistor
  pinMode(switchpin, INPUT_PULLUP);
  // After setting up the button, setup the Bounce instance :
  debouncer.attach(switchpin);
  debouncer.interval(50);
  //Setup the LED :
  pinMode(ledPin, OUTPUT);
}
//======================================================================================
void loop()
{
  // to Excel step two, and that's it!!
  //  Serial.print("DATA,DATE,TIME,"); all a bit flakey.....so not used at mo.
  //Serial.print("DATA,");

  // Charge controller first
  //mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
  //AddressRegistry_3100()
  unsigned long started_waiting_at = millis();  //Get start time
  // try to get registers
  result = Tracer_modbus.readInputRegisters(0x3100, 16);// worked with 16, 32 byte buffer
  // Having got registers, check if success. no, then wait 50mls, and get it again until success
  // didnt work without this minor delay.
  // maybe use do while
  while (!result == 0)
  {
    delay(50); // give tracer moment to settle
    result = Tracer_modbus.readInputRegisters(0x3100, 16);

    if (millis() - started_waiting_at > 550 )
    { // but, if we have waited 220 mls move on.. see how this goes
      //Serial.println("No response received - timeout!");
      break;// and get out of this while loop, otherwise we could be here for ever.
    }
  }
  if (result == Tracer_modbus.ku8MBSuccess) // then send the registers to serial port, to be collected  on computer.
  {
    PanelsV = Tracer_modbus.getResponseBuffer(0x00) / 100.0f;
    Serial.print(PanelsV);
    Serial.print(",");

    PanelsC = Tracer_modbus.getResponseBuffer(0x01) / 100.0f;
    Serial.print(PanelsC);
    Serial.print(",");

    PanelsW = Tracer_modbus.getResponseBuffer(0x02) / 100.0f;
    Serial.print(PanelsW);
    Serial.print(",");

    BatteryV = Tracer_modbus.getResponseBuffer(0x04) / 100.0f;
    Serial.print(BatteryV);
    Serial.print(",");

    BatteryCC = Tracer_modbus.getResponseBuffer(0x05) / 100.0f;
    Serial.print(BatteryCC);
    Serial.print(",");

    LoadW = Tracer_modbus.getResponseBuffer(0x0E) / 100.0f;
    Serial.print(LoadW);
    Serial.print(",");
  }
  else
  {
    rs485DataReceived = false;// if we didnt get the registers after about 4 goes, then move on, but print something.
    // Serial.println("Fail readInputRegisters(0x3100, 16) "); //take this out later
    Serial.print("11.11,11.11,11.11,11.11,11.11,11.11,");
  }
  //----------------------------------------------------------------------------------
  //void AddressRegistry_311B()
  started_waiting_at = millis();  //Get start time
  result = Tracer_modbus.readInputRegisters(0x311B, 1);
  while (!result == 0)
  {
    delay(50); // give tracer moment to settle
    result = Tracer_modbus.readInputRegisters(0x311B, 1);
    if (millis() - started_waiting_at > 220 )
    { // but, if we have waited 220 mls move on.. see how this goes
      //Serial.println("No response received - timeout!");
      break;// and get out of this while loop, otherwise we could be here for ever.
    }
  }
  if (result == Tracer_modbus.ku8MBSuccess)
  {
    BatteryTemp = Tracer_modbus.getResponseBuffer(0x00) / 100.0f;
    Serial.print(BatteryTemp);
    Serial.print(",");
  }
  else
  {
    rs485DataReceived = false;
    // Serial.println("Fail readInputRegisters(0x311B, 1) ");//take this out later
    Serial.print("22.22,");
  }
  //----------------------------------------------------------------------------------
  //void AddressRegistry_3300()
  started_waiting_at = millis();  //Get start time
  result = Tracer_modbus.readInputRegisters(0x3300, 16);
  while (!result == 0)
  {
    delay(50); // give tracer moment to settle
    result = Tracer_modbus.readInputRegisters(0x3300, 16);
    if (millis() - started_waiting_at > 220 )
    { // but, if we have waited 220 mls move on.. see how this goes
      //Serial.println("No response received - timeout!");
      break;// and get out of this while loop, otherwise we could be here for ever.
    }
  }
  if (result == Tracer_modbus.ku8MBSuccess)
  {
    MaxPanelVToday = Tracer_modbus.getResponseBuffer(0x00) / 100.0f;
    Serial.print(MaxPanelVToday);
    Serial.print(",");

    MaxBatVToday = Tracer_modbus.getResponseBuffer(0x02) / 100.0f;
    Serial.print(MaxBatVToday);
    Serial.print(",");

    MinBatVToday = Tracer_modbus.getResponseBuffer(0x03) / 100.0f;
    Serial.print(MinBatVToday);
    Serial.print(",");

    GeneratedEnergyToday = Tracer_modbus.getResponseBuffer(0x0C) / 100.0f;
    Serial.print(GeneratedEnergyToday);
    Serial.print(",");
  }
  else
  {
    rs485DataReceived = false;
    // Serial.println("Fail readInputRegisters(0x3300, 16)");//take this out later
    Serial.print("33.33,33.33,33.33,33.33,");
  }
  //----------------------------------------------------------------------------------------------
  //void AddressRegistry_9000()
  started_waiting_at = millis();  //Get start time
  result = Tracer_modbus.readHoldingRegisters(0x9000, 15);
  while (!result == 0)
  {
    delay(50); // give tracer moment to settle
    result = Tracer_modbus.readHoldingRegisters(0x9000, 15);
    if (millis() - started_waiting_at > 220 )
    { // but, if we have waited 220 mls move on.. see how this goes
      //Serial.println("No response received - timeout!");
      break;// and get out of this while loop, otherwise we could be here for ever.
    }
  }
  if (result == Tracer_modbus.ku8MBSuccess)
  {
    HVD = Tracer_modbus.getResponseBuffer(0x03) / 100.0f;
    Serial.print(HVD);
    Serial.print(",");

    CLV = Tracer_modbus.getResponseBuffer(0x04) / 100.0f;
    Serial.print(CLV);
    Serial.print(",");

    OVR = Tracer_modbus.getResponseBuffer(0x05) / 100.0f;
    Serial.print(OVR);
    Serial.print(",");

    EQUV = Tracer_modbus.getResponseBuffer(0x06) / 100.0f;
    Serial.print(EQUV);
    Serial.print(",");

    BoostV = Tracer_modbus.getResponseBuffer(0x07) / 100.0f;
    Serial.print(BoostV);
    Serial.print(",");

    FloatV = Tracer_modbus.getResponseBuffer(0x08) / 100.0f;
    Serial.print(FloatV);
    Serial.print(",");

    BoostRecV = Tracer_modbus.getResponseBuffer(0x09) / 100.0f;
    Serial.print(BoostRecV);
    Serial.print(",");

    LVR = Tracer_modbus.getResponseBuffer(0x0A) / 100.0f;
    Serial.print(LVR);
    Serial.print(",");

    LVD = Tracer_modbus.getResponseBuffer(0x0D) / 100.0f;
    Serial.print(LVD);
    Serial.print(",");
  }
  else
  {
    rs485DataReceived = false;
    // Serial.println("Fail readHoldingRegisters(0x9000, 16)");//take this out later
    Serial.print("44.44,44.44,44.44,44.44,44.44,44.44,44.44,44.44,44.44,");
  }
  //----------------------------------------------------------------------------------------------
  //                            LOADON ????
  //void AddressRegistry_0()
  //bool x = true;
  //printf("%d\n", x); // prints 1
  //result = Tracer_modbus.readHoldingRegisters(0x906A, 1); load on but v=0??try
  // result = Tracer_modbus.readCoils(0x3, 1);

  started_waiting_at = millis();  //Get start time
  result = Tracer_modbus.readCoils(0x2, 1);
  while (!result == 0)
  {
    delay(50); // give tracer moment to settle
    result = Tracer_modbus.readCoils(0x2, 1);
    if (millis() - started_waiting_at > 220 )
    { // but, if we have waited 220 mls move on.. see how this goes
      //Serial.println("No response received - timeout!");
      break;// and get out of this while loop, otherwise we could be here for ever.
    }
  }
  if (result == Tracer_modbus.ku8MBSuccess)
  {
    //https://github.com/4-20ma/ModbusMaster/blob/master/examples/RS485_HalfDuplex/RS485_HalfDuplex.ino
    LoadOn = Tracer_modbus.getResponseBuffer(0x00);
    //xxxxxxxxxxxxxxxxxxxxxx switch check and action xxxxxxxxxxxxxxxxxxxxxxxxxx
    // Update the Bounce instance :
    debouncer.update();
    // Call code if Bounce fell (transition from HIGH to LOW) :
    if ( debouncer.fell() )
    {
      // Toggle the value of loadon, because we want to change it
      // that's the purpose of the toggle switch, having found out what it is.. in LoadOn.
      LoadOn = !LoadOn;
      // Toggle the coil at address 0x0002 (Manual Load Control)
      result = Tracer_modbus.writeSingleCoil(0x2, LoadOn);
    }
    // If we are here we have had a good read signal so LOadOn will be a sure indicator of Load On/Off
    // So let the LED reflect this state, even if we have toggled it.. might need a while etc check here for result to = 0

    digitalWrite(ledPin, LoadOn);
  
  //xxxxxxxxxxxxxxxxxxxxxxxx end of switch check and action xxxxxxxxxxxxxxxxxxxxxxxxxxx
  Serial.print(LoadOn);// or...
  //Serial.print(Tracer_modbus.getResponseBuffer(0x00));
  Serial.print(",");
}
else
{
  rs485DataReceived = false;
  // Serial.println("Fail readHoldingRegisters(0x906A, 1) ");//take this out later
  Serial.print("55.55,");
}
//----------------------------------------------------------------------------------
//void AddressRegistry_3201() ????????????? Charging Status (Read Only)
//D3-D2 00=No charge, 01/1=Float, 10/2=Boost, 11/3=Equalisation
started_waiting_at = millis();  //Get start time
result = Tracer_modbus.readInputRegisters(0x3201, 1);
while (!result == 0)
{
  delay(50); // give tracer moment to settle
  result = Tracer_modbus.readInputRegisters(0x3201, 1);
  if (millis() - started_waiting_at > 220 )
  { // but, if we have waited 220 mls move on.. se how this goes
    //Serial.println("No response received - timeout!");
    break;// and get out of this while loop, otherwise we could be here for ever.
  }
}
if (result == Tracer_modbus.ku8MBSuccess)
{
  ChrgStatus = Tracer_modbus.getResponseBuffer(0x00);
  ChrgStatus = ChrgStatus >> 2;// shift right 2 places to get bits D3 D2 into D1 D0
  ChrgStatus = ChrgStatus & 3; // and it, bit wise with, 0000000000000011, to get just D1 D0, loose th rest
  Serial.print(ChrgStatus);
  Serial.print(",");
}
else
{
  rs485DataReceived = false;
  //Serial.println("Fail readInputRegisters(0x3201, 1) ");//take this out later
  Serial.print("66.66,");
}

//----------------------------------------------------------------------------------
// and the sdm220 meter
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//------------------------------------------------------------------------------------------------
//void AddressRegistry_0()  2

result = sdm220_modbus.readInputRegisters(0x0, 2);
if (result == sdm220_modbus.ku8MBSuccess)
{
  signed long MtrV1 = sdm220_modbus.getResponseBuffer(0x0);
  word MtrV2 = sdm220_modbus.getResponseBuffer(0x1);
  byte msb3 = MtrV1;
  byte msb4 = MtrV1 >> 8;
  byte msb1 = MtrV2;
  byte msb2 = MtrV2 >> 8;
  ifloat f;
  f.bytes[3] = msb4;
  f.bytes[2] = msb3;
  f.bytes[1] = msb2;
  f.bytes[0] = msb1;
  Serial.print(f.val, 0 );
  Serial.print(",");
}
else
{
  rs485DataReceived = false;
  // Serial.println("Fail sdm220_modbus.readInputRegisters(0x0, 2); ");
  Serial.print("00.00,");
}
//------------------------------------------------------------------------------------------------
//void AddressRegistry_0()  6

result = sdm220_modbus.readInputRegisters(0x6, 2);
if (result == sdm220_modbus.ku8MBSuccess)
{
  signed long MtrC1 = sdm220_modbus.getResponseBuffer(0x0);
  word MtrC2 = sdm220_modbus.getResponseBuffer(0x1);
  byte msb3 = MtrC1;
  byte msb4 = MtrC1 >> 8;
  byte msb1 = MtrC2;
  byte msb2 = MtrC2 >> 8;
  ifloat f;
  f.bytes[3] = msb4;
  f.bytes[2] = msb3;
  f.bytes[1] = msb2;
  f.bytes[0] = msb1;
  Serial.print(f.val, 2 );
  Serial.print(",");
}
else
{
  rs485DataReceived = false;
  // Serial.println("Fail sdm220_modbus.readInputRegisters(0x6, 2); ");
  Serial.print("00.00,");
}
//------------------------------------------------------------------------------------------------
//void AddressRegistry_0()  12, hex 0C.

  result = sdm220_modbus.readInputRegisters(0x0C, 2);
if (result == sdm220_modbus.ku8MBSuccess)
{
  signed long MtrW1 = sdm220_modbus.getResponseBuffer(0x0);
  word MtrW2 = sdm220_modbus.getResponseBuffer(0x1);
  byte msb3 = MtrW1;
  byte msb4 = MtrW1 >> 8;
  byte msb1 = MtrW2;
  byte msb2 = MtrW2 >> 8;
  ifloat f;
  f.bytes[3] = msb4;
  f.bytes[2] = msb3;
  f.bytes[1] = msb2;
  f.bytes[0] = msb1;
  Serial.print(f.val, 0 );
  Serial.print(",");
}
else
{
  rs485DataReceived = false;
  // Serial.println("Fail sdm220_modbus.readInputRegisters(0x12, 2); ");
  Serial.print("00.00,");
}
//----------------------------------------------------------------------------------
//void AddressRegistry_0()   48 Import active energy

result = sdm220_modbus.readInputRegisters(0x48, 2);
if (result == sdm220_modbus.ku8MBSuccess)
{
  signed long MtrKWh1 = sdm220_modbus.getResponseBuffer(0x0);
  word MtrKWh2 = sdm220_modbus.getResponseBuffer(0x1);
  byte msb3 = MtrKWh1;
  byte msb4 = MtrKWh1 >> 8;
  byte msb1 = MtrKWh2;
  byte msb2 = MtrKWh2 >> 8;
  ifloat f;
  f.bytes[3] = msb4;
  f.bytes[2] = msb3;
  f.bytes[1] = msb2;
  f.bytes[0] = msb1;
  Serial.print(f.val, 3 );
  Serial.print(",");
}
else
{
  rs485DataReceived = false;
  // Serial.println("Fail sdm220_modbus.readInputRegisters(0x48, 2); ");
  Serial.print("00.00,");
}

//----------------------------------------------------------------------------------
//void AddressRegistry_0()   4A Export active energy

result = sdm220_modbus.readInputRegisters(0x4A, 2);
if (result == sdm220_modbus.ku8MBSuccess)
{
  signed long MtrEKWh1 = sdm220_modbus.getResponseBuffer(0x0);
  word MtrEKWh2 = sdm220_modbus.getResponseBuffer(0x1);
  byte msb3 = MtrEKWh1;
  byte msb4 = MtrEKWh1 >> 8;
  byte msb1 = MtrEKWh2;
  byte msb2 = MtrEKWh2 >> 8;
  ifloat f;
  f.bytes[3] = msb4;
  f.bytes[2] = msb3;
  f.bytes[1] = msb2;
  f.bytes[0] = msb1;
  Serial.print(f.val, 3 );
  Serial.print(",");
}
else
{
  rs485DataReceived = false;
  // Serial.println("Fail sdm220_modbus.readInputRegisters(0x4A, 2); ");
  Serial.print("00.00,");
}
//----------------------------------------------------------------------------------

//rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr
// now get sensor data from te roof radio.
//rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr
if (downradio.available())
{
  Serial.print("  ");
  char sensordata[36] = ""; // could have..should have been a different type of var.//was 16 to changed  24 and 36 no diff
  while (downradio.available()) // while there is still data incoming
  {
    downradio.read(&sensordata, sizeof(sensordata));// then get it
    // this data in text will be picked up via CoolTerm
  }
  Serial.print(sensordata);
}
else
{
  //Serial.print("2");// shows else working
  // to be removed later
  // this shows prog continues if no signal/data from transmitter
  // Serial.println("No response received - timeout!");
  // delay(3000);// put a delay , it all goes too fast
}
//  checkForLoadONOFF();


//rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr
// new line, what ever happens
Serial.println("");
// How often do I want this data? Every 5,,10 seconds??
delay(5000);
// dont want to be over run with data!
}

/*
   void checkForLoadONOFF()()
   Switch for load on/off
   Unlike pinMode(INPUT), there is no pull-down resistor necessary. An internal
  20K-ohm resistor is pulled to 5V. This configuration causes the input to
  read HIGH when the switch is open, and LOW when it is closed.
  Momentary switch attached from pin 2 to ground
   Built-in LED on pin 13
   http://www.arduino.cc/en/Tutorial/InputPullupSerial
   https://github.com/thomasfredericks/Bounce2 this one
  #include <Bounce2.h>
  const int switchpin = 2;    // the number of the pushbutton pin
  const int ledPin = 13;      // the number of the LED pin to show Oad on/off
  int ledState = HIGH;         // the current state of the output pin.. depends on loadon/off state
  // Instantiate a Bounce object :
  Bounce debouncer = Bounce();
  void setup()
  {
  //configure switchpin2 as an input and enable the internal pull-up resistor
  pinMode(switchpin, INPUT_PULLUP);
   // After setting up the button, setup the Bounce instance :
  debouncer.attach(BUTTON_PIN);
  debouncer.interval(500);

  Setup the LED :
  pinMode(ledPin, OUTPUT);
  set initial LED state
  digitalWrite(ledPin, ledState);
  }
  void loop()
  {
  // Update the Bounce instance :
   debouncer.update();

   // Call code if Bounce fell (transition from HIGH to LOW) :
   if ( debouncer.fell() )
   {
    // Toggle the coil at address 0x0002 (Manual Load Control)
    result = node.writeSingleCoil(0x0002, state);
      // Toggle LED state :
     ledState = !ledState;
     digitalWrite(LED_PIN,ledState);
     }
  }

      // only toggle the LED if the new button state is HIGH
     // if (buttonState == HIGH) {
      //  ledState = !ledState;
        ledState = LoadOn
        digitalWrite(ledPin, ledState);
      }
  }




  // Keep in mind the pullup means the pushbutton's
  // logic is inverted. It goes HIGH when it's open,
  // and LOW when it's pressed. Turn on pin 13 when the
  // button's pressed, and off when it's not:
  if (sensorVal == HIGH) {
    digitalWrite(ledPin, LOW);
  } else {
    digitalWrite(ledPin, HIGH);
  }
  }




*/


