/* Midi Glock - Mike Cook April 2008
    based on code by kuki
   -----------------
   listen for MIDI serial data, and fire solenoids for individual notes using either direct PWM
   control or using an externl 16 channel servo controller

  #####################################################################################################################################################

  HARDWARE NOTE:
  The MIDI Socket is connected to arduino RX through an opto-isolator to invert the MIDI signal
  Connect the 8 solenoids to pin2 to pin9 on your arduino and pin 13 to the drive enabling monostable.

  ####################################################################################################################################################


  For the servo controller (I2C) you can set the pin to be fully on with
  
  pwm.setPWM(pin, 4096, 0);
  
  You can set the pin to be fully off with
  
  pwm.setPWM(pin, 0, 4096);

*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#include <SoftwareSerial.h>

SoftwareSerial midiSerial(2, 3); // RX, TX

//variables setup

byte incomingByte;
byte note;
byte velocity;
int noteDown = LOW;
int state = 0; // state machine variable 0 = command waiting : 1 = note waitin : 2 = velocity waiting
int strobe = 13;   // select the pin for the monostable
int channel = 0; // MIDI channel to respond to (in this case channel 1) chnage this to change the channel number
// MIDI channel = the value in 'channel' + 1
const int drumKick = 36; // midi decimal for drum sound 1 (kick)
const int drumSnare = 38; // midi decimal for drum sound 2 (snare)

const int drumClosedHH = 42; // midi decimal for drum sound 3 (closed HH)
const int drumPedalHH = 44; // midi decimal for drum sound 4 (pedal HH)
const int drumOpenHH = 46; // midi decimal for drum sound 4 (open HH)

const int drumRide = 51; // midi decimal for drum sound 4 (ride)
const int drumCrash1 = 49; // midi decimal for drum sound 4 (crash 1)
const int drumCrash2 = 57; // midi decimal for drum sound 4 (crash 1)

const int drumTom1 = 50; // midi decimal for drum sound 4 (tom 1)
const int drumTom2 = 48; // midi decimal for drum sound 4 (tom 2)
const int drumTom3 = 45; // midi decimal for drum sound 4 (tom 3)
const int drumTom4 = 41; // midi decimal for drum sound 4 (tom 4)

const int drumCowBell = 56; // midi decimal for drum sound 4 (cow bell)
const int drumSideStick = 37; // midi decimal for drum sound 4 (side stick)
const int drumClap = 39; // midi decimal for drum sound 4 (clap)

//used for direct PWM relay control
//const int drum1_pin = 6; // 1 Relay attached to this pin
//const int drum2_pin = 9; // 2 Relay attached to this pin
//const int drum3_pin = 10; // 3 Relay attached to this pin
//const int drum4_pin = 11; // 4 Relay attached to this pin

// name channels on serco controller
uint8_t KickServo = 0;
uint8_t SnareServo = 1;
uint8_t RideServo = 2;
uint8_t CrashServo = 3;
uint8_t Tom1Servo = 4;
uint8_t Tom2Servo = 5;
uint8_t CowBellServo = 6;
uint8_t SideStickServo = 7;
//uint8_t ClapServo = 8;

void setup() {
  Serial.println("16 channel PWM test!");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(40);  // (40-1600)
  Wire.setClock(400000);
  
  pinMode(strobe, OUTPUT);  // declare the strobe pin as output
  
//  pinMode(drum1_pin, OUTPUT);       // drum trigger 1
//  pinMode(drum2_pin, OUTPUT);       // drum trigger 2
//  pinMode(drum3_pin, OUTPUT);       // drum trigger 3
//  pinMode(drum4_pin, OUTPUT);       // drum trigger 4

  state = 0;  // initilise state machine variable
  
  //start serial with MIDI baudrate 31250 or 38400 for debugging
  midiSerial.begin(31250);
  Serial.begin(38400);
  Serial.println("Start the Midi read / trigger write program");
  digitalWrite(strobe, LOW);
  
  //reset all 8 relays to 0 (there are 8 more available on controller)
 for (uint8_t pwmnum=0; pwmnum < 8; pwmnum++) {
    pwm.setPWM(pwmnum, 4096, 0);
    Serial.print(pwmnum);
    Serial.println( " ON");
    delay(100);
    pwm.setPWM(pwmnum, 0, 4096);
    Serial.print(pwmnum);
    Serial.println( " OFF");
  }
}

//loop: wait for serial data, and interpret the message
void loop () {

  if (midiSerial.available() > 0) {
    // read the incoming byte:
    incomingByte = midiSerial.read();
    if (incomingByte < 240) {
      Serial.print(incomingByte);
      Serial.print(" ");
      Serial.println(millis());
    }
    digitalWrite(strobe, LOW);   // clear any previous strobe
    switch (state)  {
      case 0:
        // look for as status-byte, our channel, note on
        if (incomingByte == (144 | channel))  {
          noteDown = HIGH;
          state = 1;
        }
        // look for as status-byte, our channel, note off
        if (incomingByte == (128 | channel))  {
          noteDown = LOW;
          state = 1;
        }

      case 1:
        // get the note to play or stop
        if (incomingByte < 128) {
          note = incomingByte;
          state = 2;
        }
        else {
          state = 0;  // reset state machine as this should be a note number
        }
        break;

      case 2:
        // get the velocity
        if (incomingByte < 128) {
          playNote(note, incomingByte, noteDown); // fire off the solenoid
        }
        state = 0;  // reset state machine to start
    }
  }
}

void playNote(byte note, byte velocity, int down) {
  // if velocity = 0 on a 'Note ON' command, treat it as a note off
  if ((down == HIGH) && (velocity == 0)) {
    down = LOW;
  }

  int PWMvelocity = map(velocity,0,127,0,4096); //convert 0-128 midi to 0-4096 PWM

//KickServo = 0;
//SnareServo = 1;
//RideServo = 2;
//CrashServo = 3;
//Tom1Servo = 4;
//Tom2Servo = 5;
//CowBellServo = 6;
//SideStickServo = 7;


if (note == drumKick) {
    
    pwm.setPWM(KickServo, PWMvelocity, 0); // pwmnum = 0
    //analogWrite(drum1_pin, velocity); // use this to use pwm on drum trigger
    //digitalWrite(drum1_pin, down); // play it if it is one of our notes
    if (down == HIGH) digitalWrite(strobe, HIGH); // strobe high for any note on
//    Serial.print(" drumKick PWMvelocity: ");
//    Serial.println(PWMvelocity);
//    Serial.print(" drumKick velocity: ");
//    Serial.println(velocity);
  }
  else if (note == drumSnare) {
    pwm.setPWM(SnareServo, PWMvelocity, 0); // pwmnum = 1
    //analogWrite(drum2_pin, velocity); // use this to use pwm on drum trigger
    //digitalWrite(drum2_pin, down); // play it if it is one of our notes
    if (down == HIGH) digitalWrite(strobe, HIGH); // strobe high for any note on
    //Serial.print(" drumSnare vel: ");
    //Serial.println(PWMvelocity);
  }
  else if (note == drumRide || note == drumClosedHH || note == drumOpenHH || note == drumPedalHH) {
    pwm.setPWM(RideServo, PWMvelocity, 0); // pwmnum = 3
    //analogWrite(drum4_pin, velocity); // use this to use pwm on drum trigger
    if (down == HIGH) digitalWrite(strobe, HIGH); // strobe high for any note on
    //Serial.print(" drumRide or drumHH (closed, open, or pedal) ");
  }
  else if (note == drumCrash1 || note == drumCrash2) {
    pwm.setPWM(CrashServo, PWMvelocity, 0); // pwmnum = 3
    //analogWrite(drum4_pin, velocity); // use this to use pwm on drum trigger
    if (down == HIGH) digitalWrite(strobe, HIGH); // strobe high for any note on
    //Serial.print(" Crash1 or Crash2 ");
  }
  else if (note == drumTom1 || note == drumTom3) {
    pwm.setPWM(Tom1Servo, PWMvelocity, 0); // pwmnum = 2
    //analogWrite(drum3_pin, velocity); // use this to use pwm on drum trigger
    //digitalWrite(drum3_pin, down); // play it if it is one of our notes
    if (down == HIGH) digitalWrite(strobe, HIGH); // strobe high for any note on
    //Serial.print(" drumTom1 or 3 ");
  }
   else if (note == drumTom2 || note == drumTom4) {
    pwm.setPWM(Tom2Servo, PWMvelocity, 0); // pwmnum = 2
    if (down == HIGH) digitalWrite(strobe, HIGH); // strobe high for any note on
    //Serial.print(" drumTom2 or 4 ");
  }
  else if (note == drumClap || note == drumSideStick) {
    pwm.setPWM(SideStickServo, PWMvelocity, 0); // pwmnum = 2
    if (down == HIGH) digitalWrite(strobe, HIGH); // strobe high for any note on
    //Serial.print(" drumClap ");
  }
  else if (note == drumCowBell) {
    pwm.setPWM(CowBellServo, PWMvelocity, 0); // pwmnum = 2
    if (down == HIGH) digitalWrite(strobe, HIGH); // strobe high for any note on
    //Serial.print(" drumCowBell ");
  }
  //Serial.println();
}
