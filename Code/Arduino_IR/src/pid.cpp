#include <Arduino.h>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include <Wire.h>
#include <math.h>
// #if DEBUGGING
  void debug(char* message)
  {
    // for(int i = 0; i < DEBUG_DEPTH; i ++) Serial.print("  ");
    Serial.println(message);
  }
  void debug(char* message, int value)
  {
    // for(int i = 0; i < DEBUG_DEPTH; i ++) Serial.print("  ");
    Serial.print(message);
    Serial.println(value);
  }
  void debug(char* message, bool value)
  {
    // for(int i = 0; i < DEBUG_DEPTH; i ++) Serial.print("  ");
    Serial.print(message);
    Serial.println(value);
  }

#define PWM_R 10 // PWM of the right motor
#define PWM_L 11 // PWM of the left motor

#define R_DirX 4 // Direction pins of the right motor
#define R_DirY 5 // Direction pins of the right motor

#define L_DirX 6 // Direction pins of the left motor
#define L_DirY 7 // Direction pins of the left motor

#define ENC_Ri  2 // Right encoder interrupt
#define ENC_Rd 12 // Right encoder direction
#define ENC_Li  3 // Left encoder interrupt
#define ENC_Ld  9 // Left encoder direction

volatile int pos_R = 0; 
volatile int pos_L = 0;

long prevT = 0;


struct Movement {
  int direction;
  int cells;
};



// const float DISTANCE_CONSTANT = 18; // Adjust this constant to map distance to wheel rotations
void move(Movement *movement);

const float DISTANCE_CONSTANT = 18; // Adjust this constant to map distance to wheel rotations
const float PULSES_CONSTANT = 13; // Adjust this constant to map distance to wheel rotations
const int NUM_MOVEMENTS = 3; // Adjust this constant to map distance to wheel rotations

Movement movements[NUM_MOVEMENTS] = {
  {0, 10},    // Example movement 1: Forward by 2 units
  {0, 10},    // Example movement 1: Forward by 2 units
  {0, 10},    // Example movement 1: Forward by 2 units
};

volatile int currentMovementIndex = 0;    // Index of the current movement being executed

struct Direction {
  bool right_wheel[2]; // Right wheel direction array
  bool left_wheel[2]; // Left wheel direction array
};

// Define the directions array and map each direction to a corresponding struct
const Direction directions[] = {
  {{1, 0}, {0, 1}},   // Forward
  {{0, 1}, {1, 0}},   // Backward
  {{0, 1}, {0, 1}},   // Right
  {{1, 0}, {1, 0}}    // Left
};

// /------------ CLASS ------------/
class SimplePID{
  private:
    float kp, kd, ki, umax;
    float eprev, eintegral;
    
  public:
    // Default initialization list
    SimplePID() : kp(1), kd(0.5), ki(0), umax(255), eprev(0.0), eintegral(0.0) {}
    
    // Set the parameters
    void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
      kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
    }

    // Evaluate the signal
    void evalu(int value, int target, float deltaT,int &pwr, Direction& dir){
        
      // error
      int e = target - value;
      
      float dedt = (e-eprev)/(deltaT);
      eintegral = eintegral + e*deltaT;
      float u = kp*e + kd*dedt + ki*eintegral;
    
      // motor power
      pwr = (int) fabs(u);
      if( pwr > umax ){
        pwr = umax;
      }
           
      // motor direction
      dir = directions[0];
      if(u<0){
        dir = directions[1];
      }
            
      // store previous error
      eprev = e;
      // return dir;
    }
    
};
// PID classes
SimplePID pid[2];

void readEncoder_R() {
  int dir = digitalRead(ENC_Rd);
  if(dir > 0) {
    pos_R++;
  }
  else {
    pos_R--;
  }
}

void readEncoder_L() {
  int dir = digitalRead(ENC_Ld);
  if(dir > 0) {
    pos_L--;
  }
  else {
    pos_L++;
  }
}

void move(const Movement& movement) {
  // int targetDistance = movement.cells * DISTANCE_CONSTANT;
  // int targetPulses = (int) targetDistance * PULSES_CONSTANT;
  int targetPulses = (int) movement.cells * PULSES_CONSTANT;
  int initialPosR = pos_R;
  int initialPosL = pos_L;
  int pwm; // Initial PWM value
  int pwr; // Initial PWM value
  // int count = 0; // Counter to taper down the PWM gradually

  // from the directions array get value
  Direction dir = directions[movement.direction];

  // Set the direction of the right motor
  digitalWrite(R_DirX, dir.right_wheel[0]);
  digitalWrite(R_DirY, dir.right_wheel[1]);

  // Set the direction of the left motor
  digitalWrite(L_DirX, dir.left_wheel[0]);
  digitalWrite(L_DirY, dir.left_wheel[1]);

  debug("Target: ", targetPulses);

  while (abs(pos_R - initialPosR) < targetPulses || abs(pos_L - initialPosL) < targetPulses) {
    // analogWrite(PWM_PIN, pwm); // Supply PWM to motors
    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;
    // // Maintain equal rotation by adjusting PWM values
    // int diff_R = abs(pos_R - initialPosR);
    // int diff_L = abs(pos_L - initialPosL);
    // if (diff_L > diff_R) {
    //   int pwm_L_adjusted = (pwm / (1 + diff_L - diff_R)) / count;
    //   analogWrite(PWM_R, pwm/count);
    //   analogWrite(PWM_L, pwm_L_adjusted);
    // } else if (diff_R > diff_L) {
    //   int pwm_R_adjusted = (pwm / (1 + diff_R - diff_L)) / count;
    //   analogWrite(PWM_R, pwm_R_adjusted);
    //   analogWrite(PWM_L, pwm/count);
    // } else {
    //   analogWrite(PWM_L, pwm/1.3);
    //   analogWrite(PWM_R, pwm/1.3);
    //   count++;
    // }
    // for(int k = 0; k < 2; k++){
      pid[0].evalu(pos_R, targetPulses, deltaT, pwr, dir);
        // Set the direction of the right motor
      digitalWrite(R_DirX, dir.right_wheel[0]);
      digitalWrite(R_DirY, dir.right_wheel[1]);

      // Set the direction of the left motor
      digitalWrite(L_DirX, dir.left_wheel[0]);
      digitalWrite(L_DirY, dir.left_wheel[1]);
      analogWrite(PWM_R, pwr);
      pid[1].evalu(pos_L, targetPulses, deltaT, pwr, dir);
            // Set the direction of the right motor
      digitalWrite(R_DirX, dir.right_wheel[0]);
      digitalWrite(R_DirY, dir.right_wheel[1]);

      // Set the direction of the left motor
      digitalWrite(L_DirX, dir.left_wheel[0]);
      digitalWrite(L_DirY, dir.left_wheel[1]);
      analogWrite(PWM_L, pwr);
      // setMotor(dir, pwr, pwm[k], in1[k], in2[k], k); // signal the motor
    // }

    // if (abs(pos_R - initialPosR) < abs(pos_L - initialPosL)) {
    //   analogWrite(PWM_L, pwm);
    //   analogWrite(PWM_R, pwm/(1+ abs(pos_R - initialPosR) - abs(pos_L - initialPosL)));
    //   // analogWrite(PWM_R, pwm - count);
    // } else if (abs(pos_L - initialPosL) < abs(pos_R - initialPosR)) {
    //   analogWrite(PWM_R, pwm);
    //   analogWrite(PWM_L, pwm/(1+ abs(pos_L - initialPosL) - abs(pos_R - initialPosR)));
    //   // analogWrite(PWM_L, pwm - count);
    // }
      // analogWrite(PWM_L, pwm);
      // analogWrite(PWM_R, pwm);

    // if (count < pwm) {
    //   count++;
    // }

    // debug("Position right: ", abs(pos_R-initialPosR));
    debug("Position right: ", abs(pos_R-initialPosR));
    // debug("Position  left: ", abs(pos_L-initialPosL));
    debug("Position  left: ", abs(pos_L-initialPosL));

    delay(200);
  }

  debug("DONE MOVE");

  // Stop the motors after reaching the target rotations by rotating opisite
  digitalWrite(R_DirX, !dir.right_wheel[0]);
  digitalWrite(R_DirY, !dir.right_wheel[1]);
  digitalWrite(L_DirX, !dir.left_wheel[0]);
  digitalWrite(L_DirY, !dir.left_wheel[1]);

  bool stopped = false;

  // while (!stopped) {
  //   int initialPosR = pos_R;
  //   int initialPosL = pos_L;
    
  //   if ((abs(pos_R - initialPosR) <= 5) && (abs(pos_L - initialPosL) <= 5)) {
  //     stopped = true;
  //   } else {  
  //     int val_R = 5*(1+abs(pos_R - initialPosR));
  //     if (val_R >= 255) {
  //       analogWrite(PWM_R, 255);
  //     } else {
  //       analogWrite(PWM_R, val_R);
  //     }
  //     int val_L = 5*(1+abs(pos_L - initialPosL));
  //     if (val_L >= 255) {
  //       analogWrite(PWM_L, 255);
  //     } else {
  //       analogWrite(PWM_L, val_L);
  //     }
  // Gradually bring the motors to a complete stop
  analogWrite(PWM_R, pwm);
  analogWrite(PWM_L, pwm);
  delay(400);

  int stop_initialPosR = pos_R;
  int stop_initialPosL = pos_L;
    
  while (abs(pos_R - stop_initialPosR) > 15 && abs(pos_L - stop_initialPosL) > 15) {
    
    stop_initialPosR = pos_R;
    stop_initialPosL = pos_L;
    
    delay(200);
    int diff_R = abs(pos_R - stop_initialPosR);
    int diff_L = abs(pos_L - stop_initialPosL);

    // Adjust the PWM values for equalization during reverse movement
    if (diff_R > diff_L) {
      int pwm_R_adjusted = pwm / (1 + diff_R - diff_L);
      analogWrite(PWM_R, pwm_R_adjusted);
      analogWrite(PWM_L, pwm);
    } else if (diff_L > diff_R) {
      int pwm_L_adjusted = pwm / (1 + diff_L - diff_R);
      analogWrite(PWM_R, pwm);
      analogWrite(PWM_L, pwm_L_adjusted);
    } else {
      analogWrite(PWM_R, pwm/2);
      analogWrite(PWM_L, pwm/2);
    }

    debug("Position before right: ", initialPosR);
    debug("Position after right: ", pos_R);
    debug("Position before left: ", initialPosL);
    debug("Position after left: ", pos_L);

  }

  debug("STOPPED");

  analogWrite(PWM_L, 0);
  analogWrite(PWM_R, 0);
  digitalWrite(R_DirX, 0);
  digitalWrite(R_DirY, 0);
  digitalWrite(L_DirX, 0);
  digitalWrite(L_DirY, 0);
  delay(3000);
  // Move to the next movement
  // currentMovementIndex++;
}


// #include <Arduino.h>

// PID classes
// SimplePID pid[NMOTORS];


// /------------ GLOBALS AND DEFINITIONS ------------/
// Define the motors
// #define NMOTORS 2
// #define M0 0
// #define M1 1
// const int enca[] = {2,3};
// const int encb[]= {12,9};
// const int pwm[] = {10,11};
// const int in1[] = {4,6};
// const int in2[] = {5,7};

// // Global variables
// long prevT = 0;
// long posPrev[] = {0,0};

// // positions
// volatile long posi[] = {0,0};


// // targets
// float target_f[] = {0,0};
// long target[] = {0,0};

// void setTarget(float t, float deltat){

//   float positionChange[2] = {0.0,0.0};
//   float pulsesPerTurn = 13; 
//   float pulsesPerMeter = pulsesPerTurn*4.8229;

//   t = fmod(t,12); // time is in seconds
//   float velocity = 0.025; // m/s

//   if(t < 8){
//     for(int k = 0; k < 2; k++){ 
//       positionChange[k] = velocity*deltat*pulsesPerMeter;
//     }
//   }
//   // else{
//   //   for(int k = 0; k < 2; k++){ 
//   //     positionChange[k] = -velocity*deltat*pulsesPerMeter; 
//   //   } 
//   // }  

//   for(int k = 0; k < 2; k++){
//     target_f[k] = target_f[k] + positionChange[k];
//   }
//   target[0] = (long) target_f[0];
//   target[1] = (long) target_f[1];
// }

// /------------ LOOP ------------/
// void loop() {

//   // time difference
//   long currT = micros();
//   float deltaT = ((float) (currT - prevT))/( 1.0e6 );
//   prevT = currT;
    
//   // set target position
//   setTarget(currT/1.0e6,deltaT);

//   // Get the current position from the follower
//   long pos[2];

//   // Read the position in an atomic block to avoid a potential misread 
//   noInterrupts(); // disable interrupts temporarily while reading
//   for(int k = 0; k < NMOTORS; k++){
//     pos[k] = posi[k];
//   }
//   interrupts(); // turn interrupts back on
// for(int i = 0; i<2; i++){
//     Serial.print(" t before: ");
//     Serial.print(target[i]);
//   }
//   Serial.println();
//   // Loop through the motors
//   for(int k = 0; k < NMOTORS; k++){
//     int pwr, dir;
//     Serial.print(k);
//     Serial.print(" #old pwr: ");
//     Serial.println(pwr);
      //  pid[k].evalu(pos[k],target[k],deltaT,pwr,dir); // compute the position
//     Serial.print("new pwr: ");
//     Serial.println(pwr);
//     setMotor(dir,pwr,pwm[k],in1[k],in2[k], k); // signal the motor
//   }

//   for(int i = 0; i<2; i++){
//     Serial.print(" t after:  ");
//     Serial.print(target[i]);
//   }
//   Serial.println();
//   // for(int i = 0; i<2; i++){
//   //   Serial.print(pos[i]);
//   //   Serial.print("    p");
//   // }
//   // Serial.println();
  
// }

void setup() {
  Serial.begin(9600);

  pinMode(PWM_R, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(R_DirX, OUTPUT);
  pinMode(R_DirY, OUTPUT);
  pinMode(L_DirX, OUTPUT);
  pinMode(L_DirY, OUTPUT);
  pinMode(ENC_Ri, INPUT);
  pinMode(ENC_Rd, INPUT);
  pinMode(ENC_Li, INPUT);
  pinMode(ENC_Ld, INPUT);

  pid[0].setParams(10,0,0,255);
  pid[1].setParams(10,0,0,255);

  attachInterrupt(digitalPinToInterrupt(ENC_Ri), readEncoder_R, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_Li), readEncoder_L, RISING);
}

void loop() {
  // if (currentMovementIndex < sizeof(movements) / sizeof(*Movement)) {
  if (currentMovementIndex < NUM_MOVEMENTS) {
    Movement currentMovement = movements[currentMovementIndex];
    debug("Executing move: ", currentMovementIndex);
    move(currentMovement);
    currentMovementIndex++;
  }
}
