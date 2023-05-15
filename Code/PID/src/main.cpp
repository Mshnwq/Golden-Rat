// #include <Arduino.h>
// /*
//   Created by: Jay Weeks
//   On date:    27-Sep-2016

//   Purpose:
//     Demonstrate functions for safely controlling the Pmod HB5.
//     This version uses irregular functions that are designed to
//     be called at arbitrary times in the program.
//     These functions will not rely on a state machine, but will
//     halt the program while they execute.
  
//   Functions:
//     loop()
//       Sweeps through a range of speeds, reverses direction, and
//       repeats.
//       Uses the changeDir() and setSpd() functions.
//     changeDir()
//       Stops the motor.
//       Delays 10 milliseconds. (This is dead time for the
//       microcontroller.)
//       Inverts the direction
//       Delays 10 milliseconds.
//       Restarts the motor at the speed it was going previously.
//     setDir()
//       Stops the motor.
//       Delays 10 milliseconds. (This is dead time for the
//       microcontroller.)
//       Sets the direction to the new direction.
//       Delays 10 milliseconds.
//       Restarts the motor at the speed it was going previously.
//     setSpd()
//       Truncates the speed to an acceptable range (0 to 255).
//       Sets the new speed.

//   Notes:
//     This program is written for use with the Pmod HB5 and ChipKit
//     DP32. However, it should work just fine on other boards, so
//     long as you're careful with your wiring.

//     HB5           DP32
//     Pin 1 (DIR)   Pin 7 (GPIO)
//     Pin 2 (EN)    Pin 6 (OC5)
//     Pin 3 (SA)    (unconnected)
//     Pin 4 (SB)    (unconnected)
//     Pin 5 (GND)   GND
//     Pin 6 (VCC)   3V3

//     The HB5 is also connected to an external power source. A pack
//     of 4 very used AA batteries, in my case, but 
    
// */
// #define DEBUGGING true
// /*
//  * Use this to enable debugging.
//  * This will send debugging messages to serial.
//  */

// #define DIR_X 4 // Change this to whatever GPIO pin you want
// #define DIR_Y 5 // Change this to whatever GPIO pin you want
// #define EN   10 // This needs to be a PWM pin

// #define INVERT false
// /*
//  * Use this to invert the direction polarity.
//  * If, when given a positive speed, your motor doesn't turn in
//  * what you have decided is the positive direction, change this
//  * to false, and it'll change the direction your motor spins
//  * when given a positive speed.
//  * 
//  * Useful if you need to make two wheels of a robot spin in
//  * opposite directions to make the robot move forward.
//  */

// #if DEBUGGING
//   int DEBUG_DEPTH = 0;
//   void debug(char* message);
//   void debug(char* message, int value);
//   void debug(char* message, bool value);
// #endif

// void changeDir();
// void setDir(bool newDir_X, bool newDir_Y);
// int setSpd(int newSpeed);

// bool currentDir_X = 0;
// bool currentDir_Y = 0;
// int currentSpeed = 0;

// void setup()
// {
//   #if DEBUGGING
//     Serial.begin(9600);
//     DEBUG_DEPTH = 0;
//   #endif
  
//   pinMode(DIR_X, OUTPUT);
//   pinMode(DIR_Y, OUTPUT);
//   pinMode(EN, OUTPUT);
  
//   // Initialize our direction and speed to 0
//   setSpd(currentSpeed);
//   setDir(currentDir_X, currentDir_Y);
//   delay(10);   // Wait for things to settle
  
//   #if DEBUGGING
//     delay(5000); // Gimme time to start serial
//   #endif
// }

// void loop()
// {
//   // Bring our speed up
//   #if DEBUGGING
//    Serial.println("Bringing speed up from zero");
//    Serial.println("Bringing speed up from zero");
//     DEBUG_DEPTH ++;
//   #endif
//   for (int i = 0; i < 255; i += 5)
//   {
//     #if DEBUGGING
//      Serial.print("i: ");
//      Serial.println(i);
//     #endif
//     setSpd(i);
//     delay(10);
//     #if DEBUGGING
//       delay(490);
//     #endif
//   }
  
//   // Bring it down
//   #if DEBUGGING
//     DEBUG_DEPTH --;
//    Serial.println("Bringing speed down from 255");
//    Serial.println("Bringing speed down from 255");
//     DEBUG_DEPTH ++;
//   #endif
//   for (int i = 255; i > -255; i -= 5)
//   {
//     #if DEBUGGING
//      Serial.print("i: ");
//      Serial.print(i);
//     #endif
//     setSpd(i);
//     delay(10);
//     #if DEBUGGING
//       delay(490);
//     #endif
//   }
  
//   // Bring it back up again
//   #if DEBUGGING
//     DEBUG_DEPTH --;
//    Serial.println("Bringing speed up from -255");
//    Serial.println("Bringing speed up from -255");
//     DEBUG_DEPTH ++;
//   #endif
//   for (int i = -255; i < 0; i += 5)
//   {
//     #if DEBUGGING
//      Serial.print("i: ");
//      Serial.print(i);
//     #endif
//     setSpd(i);
//     delay(10);
//     #if DEBUGGING
//       delay(490);
//     #endif
//   }
//   #if DEBUGGING
//     DEBUG_DEPTH --;
//   #endif
// }

// void changeDir()
// {
//   #if DEBUGGING
//    Serial.println("Entering changeDir()");
//     DEBUG_DEPTH ++;
//    Serial.print("currentDir_X: ");
//    Serial.println(currentDir_X);
//    Serial.print("currentDir_Y: ");
//    Serial.println(currentDir_Y);
//    Serial.println("Changing direction");
//   #endif

//   int spd = 0;
  
//   // Stop our motor so we can change direction
//   analogWrite(EN, 0);
//   delay(5);  // Wait a bit for things to settle

//   // Change our direction
//   currentDir_X = !currentDir_X;
//   currentDir_Y = !currentDir_Y;
//   digitalWrite(DIR_X, (currentDir_X ^ INVERT));
//   digitalWrite(DIR_Y, (currentDir_Y ^ INVERT));
//   delay(5);  // Wait a bit for things to settle

//   // Start our motor back up again
//   spd = currentSpeed;
//   if(spd < 0) spd = -spd;
//   analogWrite(EN, spd);
  
//   #if DEBUGGING
//    Serial.print("currentDir_X: ");
//    Serial.println(currentDir_X);
//    Serial.print("currentDir_Y: ");
//    Serial.println(currentDir_Y);
//     DEBUG_DEPTH --;
//   #endif
  
// //   return currentDir;
// }

// void setDir(bool newDir_X, bool newDir_Y)
// {
//   #if DEBUGGING
//    Serial.println("Entering setDir()");
//     DEBUG_DEPTH ++  ;
//    Serial.print("currentDir_X: ");
//    Serial.println(currentDir_X);
//    Serial.print("currentDir_Y: ");
//    Serial.println(currentDir_Y);
//    Serial.println("Setting direction");
//   #endif

//   int spd = 0;
  
//   // Stop our motor so we can change direction
//   analogWrite(EN, 0);
//   delay(5);  // Wait a bit for things to settle

//   // Change our direction
//   currentDir_X = newDir_X;
//   currentDir_Y = newDir_Y;
//   digitalWrite(DIR_X, (currentDir_X ^ INVERT));
//   digitalWrite(DIR_Y, (currentDir_Y ^ INVERT));
//   delay(5);  // Wait a bit for things to settle

//   // Start our motor back up again
//   spd = currentSpeed;
//   if(spd < 0) spd = -spd;
//   analogWrite(EN, spd);

//   #if DEBUGGING
//    Serial.print("currentDir_X: ");
//    Serial.println(currentDir_X);
//    Serial.print("currentDir_Y: ");
//    Serial.println(currentDir_Y);
//    DEBUG_DEPTH --;
//   #endif
  
// //   return currentDir;
// }

// int setSpd(int newSpeed)
// {
//   #if DEBUGGING
//    Serial.println("Entering setSpd()");
//    Serial.println("Entering setSpd()");
//     DEBUG_DEPTH ++;
//    Serial.print("currentDir_X: ");
//    Serial.println(currentDir_X);
//    Serial.print("currentDir_Y: ");
//    Serial.println(currentDir_Y);
//    Serial.print("currentSpeed: ");
//    Serial.println(currentSpeed);
//    Serial.print("newSpeed: ");
//    Serial.println(newSpeed);
//    Serial.println("Direction boolean:");
//     DEBUG_DEPTH ++;
//    Serial.print("(newSpeed > 0) ^ (currentDir_X): ");
//    Serial.println((newSpeed > 0) ^ (currentDir_X));
//    Serial.print("(newSpeed > 0) ^ (currentDir_Y): ");
//    Serial.println((newSpeed > 0) ^ (currentDir_Y));
//     DEBUG_DEPTH --;
//    Serial.println("Setting speed");
//   #endif
  
//   // Ensure that new speed is within bounds
//   if(newSpeed > 255) newSpeed = 255;
//   else if (newSpeed < -255) newSpeed = -255;
  
//   // Make sure the new direction and the current direction agree
//   if ((newSpeed > 0) ^ (currentDir_X) && !((newSpeed > 0) ^ (currentDir_Y))) {
//      changeDir();
//   }
//   // Set our new speed
//   currentSpeed = newSpeed;
//   // Get its absolute value
//   if (newSpeed < 0) newSpeed = -newSpeed;
//   // Set our pin to output the new speed
//   analogWrite(EN, newSpeed);
  
//   #if DEBUGGING
//    Serial.println("currentDir_X: ", currentDir_X);
//    Serial.println("currentDir_X: ", currentDir_X);
//    Serial.println("currentDir_Y: ", currentDir_Y);
//    Serial.println("currentDir_Y: ", currentDir_Y);
//    Serial.println("currentSpeed: ", currentSpeed);
//    Serial.println("currentSpeed: ", currentSpeed);
//    Serial.println("newSpeed: ", newSpeed);
//    Serial.println("newSpeed: ", newSpeed);
//     DEBUG_DEPTH --;
//   #endif
  
//   return currentSpeed;
// }

// #if DEBUGGING
//   void ;
//   S(char* message)
//   {
//     for(int i = 0; i < DEBUG_DEPTH; i ++) Serial.print("  ");
//     Serial.println(message);
//     Serial.println(message);
//   }
//   voidSerial.println;
//   S(char* message, int value)
//   {
//     for(int i = 0; i < DEBUG_DEPTH; i ++) Serial.print("  ");
//     Serial.print(message);
//     Serial.println(value);
//     Serial.println(value);
//   }
//   voidSerial.println;
//   S(char* message, bool value)
//   {
//     for(int i = 0; i < DEBUG_DEPTH; i ++) Serial.print("  ");
//     Serial.print(message);
//     Serial.println(value);
//     Serial.println(value);
//   }
// #endif




// // I hate typing at the very bottom of the screen, don't you?