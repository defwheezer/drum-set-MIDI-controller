MIDI channels of interest from drum machine:

- const int drumKick = 36; // midi decimal for drum sound 1 (kick)
- const int drumSnare = 38; // midi decimal for drum sound 2 (snare)
 
- const int drumClosedHH = 42; // midi decimal for drum sound 3 (closed HH)
- const int drumPedalHH = 44; // midi decimal for drum sound 4 (pedal HH)
- const int drumOpenHH = 46; // midi decimal for drum sound 5 (open HH)
 
- const int drumRide = 51; // midi decimal for drum sound 6 (ride)
- const int drumCrash1 = 49; // midi decimal for drum sound 7 (crash 1)
- const int drumCrash2 = 57; // midi decimal for drum sound 8 (crash 1)
 
- const int drumTom1 = 50; // midi decimal for drum sound 9 (tom 1)
- const int drumTom2 = 48; // midi decimal for drum sound 10 (tom 2)
- const int drumTom3 = 45; // midi decimal for drum sound 11 (tom 3)
- const int drumTom4 = 41; // midi decimal for drum sound 12 (tom 4)

- const int drumCowBell = 56; // midi decimal for drum sound 13 (cow bell)
- const int drumSideStick = 37; // midi decimal for drum sound 14 (side stick)
- const int drumClap = 39; // midi decimal for drum sound 15 (clap)

Channel assignement when 16 channel PMW controller (I only use 8 channels: "0-7")

- KickServo = 0;
- SnareServo = 1;
- RideServo = 2;
- CrashServo = 3;
- Tom1Servo = 4;
- Tom2Servo = 5;
- CowBellServo = 6;
- SideStickServo = 7;
