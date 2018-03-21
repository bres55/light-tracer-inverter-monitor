// 2/3/2018
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
  Control, and indicators of states.
  SCT013 30A
*/

#include <SPI.h> // Comes with Arduino IDE
#include <RF24.h> // Download and Install...https://github.com/tmrh20/RF24/
#include <Bounce2.h> //https://github.com/thomasfredericks/Bounce2/wiki
#include <EEPROM.h>
#include "EmonLib.h"                   // Include Emon Library
#include <ModbusMaster.h> // https://github.com/4-20ma/ModbusMaster

const uint8_t switchpin =          2;    // the number of the pushbutton pin
const uint8_t PowerOnOffLedPin =  12;    // the number of the LED pin to show My power switch is on/off
const uint8_t PowerInLedPin =     13;    // the number of the LED pin to show Power is coming into house, or not
//const uint8_t GTILOnOffLedPin = 14;    // the number of the LED pin to show GTIL on/off.. not implimented yet

const int On = 1;
const int Off = 0;

float PanelsV, PanelsC, PanelsW, BatteryV, BatteryCC, LoadW, BatteryTemp, MaxPanelVToday, MaxBatVToday;
float MinBatVToday, GeneratedEnergyToday, HVD, CLV, OVR, EQUV, BoostV, FloatV, BoostRecV, LVR, LVD;
word ChrgStatus;    //1=on 0=off,,,,D3-D2 00=No charge, 01/1=Float, 10/2=Boost, 11/3=Equalisation
word DisChrgStatus; //1=on 0=off,ie we have load switched on if manual load switch is on.
bool LoadOn;
// float MtrV, MtrC, MtrW, MtrKWh1, MtrKWh2,MtrKWh3, MtrKWh;
uint8_t result;
const int Eepromaddress = 0;
byte loadflag;
double MainsInW;

// initialise ModbusMaster object
//better names
ModbusMaster Tracer_modbus; //old node
ModbusMaster sdm220_modbus; // old node2
// What we would do pre and post transmission. nothing
// tracer requires no handshaking, suspect this is why I have had to put in delays.. letting CC settle
void preTransmission() {}
void postTransmission() {}

// this is to check if we can write since rs485 is half duplex ??? not sure what this is about??
bool rs485DataReceived = true;

RF24 downradio(7, 8); // my down radio uses pins on UNO, and receiver is a Mega CE=7, CNS=8
// Only RoofT used as I dont send any info back yet
byte addresses[][6] = {"RoofT", "DownR"};

// Instantiate a Bounce object :
Bounce debouncer = Bounce();

EnergyMonitor emon1;    // Create an instance, for SCT013 current clamp

// UNION, used to re-jig float
// used to get numbers from meter
// No idea how it works, but it does.
// https://arduino.stackexchange.com/questions/38400/float-not-a-float
union ifloat {
  uint8_t bytes[4];
  float val;
};
// END UNION
//ssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssss
void setup()
{
  Serial.begin(115200); // will be sending all data to serial, for later analysis
  // Initiate the radio object
  downradio.begin();
  // Set the speed of the transmission to moderate
  // roofradio.setDataRate(RF24_2MBPS);
  downradio.setDataRate(RF24_250KBPS);
  // Use a channel unlikely to be used by Wifi
  downradio.setChannel(124);
  // Set the transmit power to highest available, it is on the roof!!
  // downradio.setPALevel(RF24_PA_MIN);
  downradio.setPALevel(RF24_PA_MAX);
  // Open a reading pipe,  using the radio.setReadingPipe() function we set the same address
  // in that way we enable the communication between the two modules.
  downradio.openReadingPipe(1, addresses[1]);
  // Now listen for a response
  downradio.startListening();

  //  Initialize Modbus communication baud rate, usually 115200
  //  Tracer---------- Charge Controller
  //  On a Mega 256, using Serial 1, Modbus slave ID 2.. my test tracer is on 2, usually 1;
  Serial1.begin(115200);
  Tracer_modbus.begin(1, Serial1);//1 tracer
  // SDM220 ---------- Meter
  // On a Mega 256, using Serial 2, Modbus slave ID 1
  Serial2.begin(2400);
  sdm220_modbus.begin(1, Serial2);//1 sdm220
  // let it --Tracer_modbus-- know what we want for pre and post transmission, we call above void,
  // which was--do nothing.
  Tracer_modbus.preTransmission(preTransmission);
  Tracer_modbus.postTransmission(postTransmission);
  sdm220_modbus.preTransmission(preTransmission);
  sdm220_modbus.postTransmission(postTransmission);

  // to excel step one---------not used at mo.
  // Serial.println("CLEARDATA"); // clears starting at row 2
  // Serial.println("CLEARSHEET"); // clears starting at row 1
  // Serial.println("LABEL,Date,Time,PanV,PanC,PanP,BatV,BatC,LdP,BatTp,MaxP,MaxB,MinB,PKwh,HVD,CLV,OVR,.......
  // .............EQUV,BSTV,FLTV,BRV,LVR,LVD,LdOn,Csts,MtrV,MtrC,MtrW,MtrKwh");

  // for load on off switch
  pinMode(switchpin, INPUT_PULLUP); //configure switchpin2 as an input and enable the internal pull-up resistor
  // After setting up the button, setup the Bounce instance :
  debouncer.attach(switchpin);
  debouncer.interval(50);
  // Setup the LED, for Power in
  pinMode(PowerInLedPin, OUTPUT);
  // Setup the LED, for Power switch on/off
  pinMode(PowerOnOffLedPin, OUTPUT);

  // For STC current monitor, SCT013 15A
  emon1.current(1, 14.4);  // Current: input pin, calibration.
}
//======================================================================================
//                 LOOP
//======================================================================================
void loop()
{
  // to Excel step two, and that's it!!
  // Serial.print("DATA,DATE,TIME,"); all a bit flakey.....so not used at mo.
  // Serial.print("DATA,");

  // Charge controller first
  // mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
  // AddressRegistry_3100()
  unsigned long started_waiting_at = millis();  //Get start time
  // try to get registers
  // Modbus function 0x04 Read Input Registers.
  result = Tracer_modbus.readInputRegisters(0x3100, 16);// worked with 16, 32 byte buffer
  // Having got registers, check if success. no, then wait 50mls, and get it again until success
  // didnt work without this minor delay.
  // maybe use do while
  while (!result == 0)
  {
    delay(50); // give tracer moment to settle
    result = Tracer_modbus.readInputRegisters(0x3100, 16);

    if (millis() - started_waiting_at > 550 )
    { // but, if we have waited 550 mls move on.. see how this goes
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
  // void AddressRegistry_311B()
  started_waiting_at = millis();  //Get start time
  // Modbus function 0x04 Read Input Registers.
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
  // ----------------------------------------------------------------------------------
  // not sure why i need a delay here, without it when no power from panels and
  // manual mode off, i lose this set of data 33.33 and the 66.66
  // 2500 worked try 500, 50, 10 ..seems to be longer than 10ms ??
  delay(10);
  // void AddressRegistry_3300()
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
  // ----------------------------------------------------------------------------------------------
  // void AddressRegistry_9000()
  started_waiting_at = millis();  //Get start time
  // Modbus function 0x03 Read Holding Registers.
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
  // ----------------------------------------------------------------------------------------------
  //                            LOADON ????
  started_waiting_at = millis();  //Get start time
  // Modbus function 0x01 Read Coils.
  // Modbus function 0x05 Write Single Coil.
  result = Tracer_modbus.readCoils(0x2, 1);
  while (!result == 0)
  {
    delay(50); // give tracer moment to settle
    result = Tracer_modbus.readCoils(0x2, 1);
    if (millis() - started_waiting_at > 220 )
    { // but, if we have waited 220 mls move on.. see how this goes
      // Serial.println("No response received - timeout!");
      break;// and get out of this while loop, otherwise we could be here for ever.
    }
  }
  if (result == Tracer_modbus.ku8MBSuccess)
  {
    // https://github.com/4-20ma/ModbusMaster/blob/master/examples/RS485_HalfDuplex/RS485_HalfDuplex.ino
    LoadOn = Tracer_modbus.getResponseBuffer(0x00);

    // get loadflag from eeprom
    loadflag = EEPROM.read(Eepromaddress);

    // xxxxxxxxxxxxxxxxxxxxxx switch check and action xxxxxxxxxxxxxxxxxxxxxxxxxx
    // Update the Bounce instance :
    debouncer.update();
    // Call code if Bounce fell (transition from HIGH to LOW) :
    if ( debouncer.fell() )
    {
      // should put this in an interupt
      // xxxx Toggle the value of loadflag, because we want to change it
      // that's the purpose of the toggle switch, having read it from eeprom.
      loadflag  = !loadflag;
      EEPROM.write(Eepromaddress, loadflag); // And save it
      LoadOn = loadflag; //change the value of LoadOn
      // Sync CC Manual control the LoadOn, with my loadflag.
      delay(50);                                                // give tracer moment to settle....try a delay , it
      result = Tracer_modbus.writeSingleCoil(0x2, LoadOn); // doesn't seem to do anything, probably a timing issue
      //like the rest of probs.  ????????????
    }
    // xxxxxxxxxxxxxxxxxxxxxxxx end of switch check and action xxxxxxxxxxxxxxxxxxxxxxxxxxx
    digitalWrite(PowerOnOffLedPin, loadflag); // and switch on LED to show on or off GREEN
    Serial.print(loadflag); // see on serial output if out of sync, if so press button, if necessary.
    Serial.print(",");

    Serial.print(LoadOn);// or...
    // Serial.print(Tracer_modbus.getResponseBuffer(0x00));
    Serial.print(",");
  }
  else
  {
    rs485DataReceived = false;
    // Serial.println("Fail readHoldingRegisters(0x906A, 1) ");//take this out later
    Serial.print("55.55,55.55,");
  }
  //----------------------------------------------------------------------------------

  // Get Charge and Discharge Status

  // not sure why i need a delay here, without it when no power from panels and
  // manual mode off, i lose this set of data 33.33 and the 66.66
  delay(100); //needed even longer delay here after getting loadflag to work with 50 delay before that!!
  // see LOADON section.

  // void AddressRegistry_3201() ????????????? Charging Status (Read Only)
  // D3-D2 00=No charge, 01/1=Float, 10/2=Boost, 11/3=Equalisation
  started_waiting_at = millis();  //Get start time
  // Modbus function 0x04 Read Input Registers.
  result = Tracer_modbus.readInputRegisters(0x3201, 2);
  while (!result == 0)
  {
    delay(50); // give tracer moment to settle
    result = Tracer_modbus.readInputRegisters(0x3201, 2);
    if (millis() - started_waiting_at > 220 )
    { // but, if we have waited 220 mls move on.. se how this goes
      // Serial.println("No response received - timeout!");
      break;// and get out of this while loop, otherwise we could be here for ever.
    }
  }
  if (result == Tracer_modbus.ku8MBSuccess)
  {
    ChrgStatus = Tracer_modbus.getResponseBuffer(0x00);
    ChrgStatus = ChrgStatus >> 2;// shift right 2 places to get bits D3 D2 into D1 D0
    ChrgStatus = ChrgStatus & 3; // AND it, bit wise with, 0000000000000011, to get just D1 D0, loose the rest

    Serial.print(ChrgStatus);
    Serial.print(",");

    // GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG
    /*   taken out while GTIL to panel, no CC used but kept in for remote on/off function. No batteries

      // If loadflag is OFF, don't want to turn LOadOn back on again here, even if power available
      // so only do chk charge status if loadflag is ON. This way loadflag, switched on by my
      // switch, controls if I want CC on or off
      // I can still switch the CC off at the CC switch, but then things will be out of sync
      // but that is ok, would need to do a button off, button on.
      // if charge status is 0, ie no power coming in from panels, then turn the GTIL off, to save GTIL power loss

      if (loadflag == On)
      {
      if (ChrgStatus == Off)
      {
        result = Tracer_modbus.writeSingleCoil(0x2, Off); // turn it off
      }
      else
      {
        // So, must be charging at, Equalisation, Boost or Float
        result = Tracer_modbus.writeSingleCoil(0x2, On); // else turn it on

        // maybe add later if gtil is on for long spells without generating.
        // but if we are charging, at, Equalisation, Boost or Float, is the gtil, generating??
        // get meter watts
        // if - value
        // turn off
        // else turn it on here

      }
      }   // if loadflag = Off, do nothing.
      // GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG
    */
    // GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG
    // get discharge staus... not really of interest, but keep for now.
    DisChrgStatus = Tracer_modbus.getResponseBuffer(0x01);
    // D0: 1 Running, 0 Standby.
    DisChrgStatus = DisChrgStatus & 1; // AND it, bit wise with, 0000000000000001, to get just D0, loose th rest
    Serial.print(DisChrgStatus);
    Serial.print(",");
  }
  else
  {
    rs485DataReceived = false;
    // Serial.println("Fail readInputRegisters(0x3201, 1) ");//take this out later
    Serial.print("66.66,66.66,");
  }

  //----------------------------------------------------------------------------------
  // and the SDM220 meter
  // mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
  //------------------------------------------------------------------------------------------------
  // no need for delays here.
  // void AddressRegistry_0()  2
  // Modbus function 0x04 Read Input Registers.
  // Meter Voltage
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
    Serial.print(f.val, 0 );// no decimal places
    Serial.print(",");

    // Get Mains Current, at input, .......SCT013 30A...............

    double Irms = emon1.calcIrms(1480);  // Calculate Irms only

    MainsInW = Irms * f.val;  // current * meter voltage, save it for print out later
    //MainsInW=Irms;         // if testing just irms
  }
  else
  {
    rs485DataReceived = false;
    // Serial.println("Fail sdm220_modbus.readInputRegisters(0x0, 2); ");
    Serial.print("00.00,");
  }
  //------------------------------------------------------------------------------------------------
  // void AddressRegistry_0()  6
  // Meter Current
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
    Serial.print(f.val, 2 ); //2 decimail places
    Serial.print(",");
  }
  else
  {
    rs485DataReceived = false;
    // Serial.println("Fail sdm220_modbus.readInputRegisters(0x6, 2); ");
    Serial.print("00.00,");
  }
  // ------------------------------------------------------------------------------------------------
  // void AddressRegistry_0()
  //      12, hex 0C.  Active power. Watts, seems best
  // or   18, hex 12.  Apparent power VoltAmps, only v * a
  // or   24. hex 18.  Reactive power VAr , not sure..low value

  // Meter Wattage
  result = sdm220_modbus.readInputRegisters(0x0c, 2);
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

    // LED light signal RED
    // So let the LED reflect if we have power going OUT from gtil ie above 9W
    if (f.val > 9)
    {
      digitalWrite(PowerInLedPin, On);
    }
    else
    {
      // otherwise we do not
      digitalWrite(PowerInLedPin, Off);
    }
    Serial.print(f.val, 0 );      // no decimal places
    Serial.print(",");

    // MainsInW=MainsInW-f.val;    // subtract GTIL wattage from MAINS IN, to get actual MAINS IN
    // the values dont work???
    Serial.print(MainsInW);       // And print the mains in wattage calculated just after meter V
    Serial.print(",");
  }
  else
  {
    rs485DataReceived = false;
    // Serial.println("Fail sdm220_modbus.readInputRegisters(0x12, 2); ");
    Serial.print("00.00,");
  }
  // ----------------------------------------------------------------------------------
  // void AddressRegistry_0()   Import active energy kwh, running total of kwh
  // Meter Kilo Watt Hours, accumulative.
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
    Serial.print(f.val, 3 ); // 3 decimal places
    Serial.print(",");
  }
  else
  {
    rs485DataReceived = false;
    // Serial.println("Fail sdm220_modbus.readInputRegisters(0x48, 2); ");
    Serial.print("00.00,");
  }

  // ----------------------------------------------------------------------------------
  // void AddressRegistry_0()   4A Export active energy
  // Not sure why I want this, keep for now
  result = sdm220_modbus.readInputRegisters(0x4a, 2);
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

  // rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr
  // now get sensor data from te roof radio.
  // rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr
  if (downradio.available())
  {
    Serial.print("  ");
    char sensordata[36] = ""; // could have..should have been a different type of var.
    // was 16 to changed  24 and 36 no diff
    while (downradio.available()) // while there is still data incoming
    {
      downradio.read(&sensordata, sizeof(sensordata));// then get it
      // this data in text will be picked up via CoolTerm
    }
    // Serial.print(sensordata);
    // decided to split sensor data, I might want to use the bh2 separately
    // https://forum.arduino.cc/index.php?topic=376296.0
    String dataString = (sensordata);
    int firstCommaIndex = dataString.indexOf(',');
    int secondCommaIndex = dataString.indexOf(',', firstCommaIndex + 1);
    int thirdCommaIndex = dataString.indexOf(',', secondCommaIndex + 1);
    String bh1750 = dataString.substring(0, firstCommaIndex);
    String bh21750 = dataString.substring(firstCommaIndex + 1, secondCommaIndex);
    String tmt6000 = dataString.substring(secondCommaIndex + 1, thirdCommaIndex);
    String dht11 = dataString.substring(thirdCommaIndex + 1);

    Serial.print(bh1750);
    Serial.print(",");
    Serial.print(bh21750);
    Serial.print(",");
    Serial.print(tmt6000);
    Serial.print(",");
    Serial.print(dht11);
  }
  else
  {
    // Serial.print("2");// shows else working
    // to be removed later
    // this shows prog continues if no signal/data from transmitter
    // Serial.println("No response received - timeout!");
    // delay(3000);// put a delay , it all goes too fast
  }

  // rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr
  // new line, what ever happens
  Serial.println("");
  // How often do I want this data? Every 5,,10 seconds??
  delay(5000);
  // dont want to be over run with data!
}
// eeeeeeeeeeeeeeeeeeeeennnnnnnnnnnnnnnnnnnnnnndddddddddddddddddddddd

