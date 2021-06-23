/* based on code by kuki
   -----------------
   listen for MIDI serial data, and fire solenoids for individual notes

  #####################################################################################################################################################

  HARDWARE NOTE:
  The MIDI Socket is connected to arduino RX through an opto-isolator to invert the MIDI signal and seperate the circuits of individual instruments.
  Connect the 8 solenoids to pin2 to pin9 on your arduino and pin 13 to the drive enabling monostable.
  
  modified to use 16-channel servo controller (Adafruit PCA9685 16-Channel Servo Driver)

  ####################################################################################################################################################


  You can set the pin to be fully on with
  
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
const int drumCrash = 49; // midi decimal for drum sound 4 (crash 1)

const int drumTom1 = 50; // midi decimal for drum sound 4 (tom 1)
const int drumTom2 = 48; // midi decimal for drum sound 4 (tom 2)
const int drumTom3 = 45; // midi decimal for drum sound 4 (tom 3)
const int drumTom4 = 41; // midi decimal for drum sound 4 (tom 4)

const int drumCowBell = 56; // midi decimal for drum sound 4 (cow bell)
const int drumSideStick = 37; // midi decimal for drum sound 4 (side stick)

const int drum1_pin = 6; // midi decimal for drum sound 1
const int drum2_pin = 9; // midi decimal for drum sound 2
const int drum3_pin = 10; // midi decimal for drum sound 3
const int drum4_pin = 11; // midi decimal for drum sound 4
//setup: declaring iputs and outputs and begin serial
void setup() {
  Serial.println("16 channel PWM test!");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(40);  // (40-1600)
  Wire.setClock(400000);
  
  pinMode(strobe, OUTPUT);  // declare the strobe pin as output
  pinMode(drum1_pin, OUTPUT);       // drum trigger 1
  pinMode(drum2_pin, OUTPUT);       // drum trigger 2
  pinMode(drum3_pin, OUTPUT);       // drum trigger 3
  pinMode(drum4_pin, OUTPUT);       // drum trigger 4
  state = 0;  // initilise state machine variable
  //start serial with MIDI baudrate 31250 or 38400 for debugging
  
  midiSerial.begin(31250);
  Serial.begin(38400);
  Serial.println("Start the awesome midi read / trigger write program");
  digitalWrite(strobe, LOW);
  //reset all relays to 0
    for (uint8_t pwmnum=0; pwmnum < 8; pwmnum++) {
    pwm.setPWM(pwmnum, 4096, 0);
    Serial.print(pwmnum);
    Serial.println( " ON");
    delay(100);
  }
  for (uint8_t pwmnum=0; pwmnum < 8; pwmnum++) {
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

  if (note == drumKick) {
    pwm.setPWM(0, PWMvelocity, 0); // pwmnum = 0
    analogWrite(drum1_pin, velocity); // use this to use pwm on drum trigger
    //digitalWrite(drum1_pin, down); // play it if it is one of our notes
    if (down == HIGH) digitalWrite(strobe, HIGH); // strobe high for any note on
//    Serial.print(" drumKick PWMvelocity: ");
//    Serial.println(PWMvelocity);
//    Serial.print(" drumKick velocity: ");
//    Serial.println(velocity);
  }
  else if (note == drumSnare) {
    pwm.setPWM(1, PWMvelocity, 0); // pwmnum = 1
    analogWrite(drum2_pin, velocity);
    //digitalWrite(drum2_pin, down); // play it if it is one of our notes
    if (down == HIGH) digitalWrite(strobe, HIGH); // strobe high for any note on
    //Serial.print(" drumSnare vel: ");
    //Serial.println(PWMvelocity);
  }
  else if (note == drumTom1 || note == drumTom2 || note == drumTom3 || note == drumTom4) {
    pwm.setPWM(2, PWMvelocity, 0); // pwmnum = 2
    analogWrite(drum3_pin, velocity);
    //digitalWrite(drum3_pin, down); // play it if it is one of our notes
    if (down == HIGH) digitalWrite(strobe, HIGH); // strobe high for any note on
    //Serial.print(" drumTom ");
    }
  else if (note == drumClosedHH || note == drumOpenHH || note == drumCrash) {
    pwm.setPWM(3, PWMvelocity, 0); // pwmnum = 3
    analogWrite(drum4_pin, velocity);
    //digitalWrite(drum4_pin, down); // play it if it is one of our notes
    if (down == HIGH) digitalWrite(strobe, HIGH); // strobe high for any note on
    //Serial.print(" drumHH or Crash ");
  }
  Serial.println();
}
