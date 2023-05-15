// #include "Arduino.h" // This library allows you to communicate with I2C devices.

// // #include <Adafruit_MPU6050.h>
// // #include <Adafruit_Sensor.h>
// // #include <Wire.h>

// // Adafruit_MPU6050 mpu;

// // void setup(void) {
// //   Serial.begin(115200);
// //   while (!Serial)
// //     delay(10); // will pause Zero, Leonardo, etc until serial console opens

// //   Serial.println("Adafruit MPU6050 test!");

// //   // Try to initialize!
// //   if (!mpu.begin()) {
// //     Serial.println("Failed to find MPU6050 chip");
// //     while (1) {
// //       delay(10);
// //     }
// //   }
// //   Serial.println("MPU6050 Found!");

// //   //setupt motion detection
// //   mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
// //   mpu.setMotionDetectionThreshold(1);
// //   mpu.setMotionDetectionDuration(20);
// //   mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
// //   mpu.setInterruptPinPolarity(true);
// //   mpu.setMotionInterrupt(true);

// //   Serial.println("");
// //   delay(100);
// // }

// // void loop() {

// //   if(mpu.getMotionInterruptStatus()) {
// //     /* Get new sensor events with the readings */
// //     sensors_event_t a, g, temp;
// //     mpu.getEvent(&a, &g, &temp);

// //     /* Print out the values */
// //     Serial.print("AccelX:");
// //     Serial.print(a.acceleration.x);
// //     Serial.print(",");
// //     Serial.print("AccelY:");
// //     Serial.print(a.acceleration.y);
// //     Serial.print(",");
// //     Serial.print("AccelZ:");
// //     Serial.print(a.acceleration.z);
// //     Serial.print(", ");
// //     Serial.print("GyroX:");
// //     Serial.print(g.gyro.x);
// //     Serial.print(",");
// //     Serial.print("GyroY:");
// //     Serial.print(g.gyro.y);
// //     Serial.print(",");
// //     Serial.print("GyroZ:");
// //     Serial.print(g.gyro.z);
// //     Serial.println("");
// //   }

// //   delay(10);
// // }
// // // #include "Wire.h" // This library allows you to communicate with I2C devices.

// // // const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.

// // // int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
// // // int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
// // // int16_t temperature; // variables for temperature data

// // // char tmp_str[7]; // temporary variable used in convert function

// // // char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
// // //   sprintf(tmp_str, "%6d", i);
// // //   return tmp_str;
// // // }
// // // // #include<Wire.h>
// // // // const int MPU=0x68; 
// // // // int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

// // // // void  setup(){
// // // //   Wire.begin();
// // // //   Wire.beginTransmission(MPU);
// // // //   Wire.write(0x6B);  
// // // //   Wire.write(0);    
// // // //   Wire.endTransmission(true);
// // // //   Serial.begin(9600);
// // // // }

// // // // void  loop(){
// // // //   Wire.beginTransmission(MPU);
// // // //   Wire.write(0x3B);  
// // // //   Wire.endTransmission(false);
// // // //   Wire.requestFrom(MPU,12,true);  
// // // //   AcX=Wire.read()<<8|Wire.read();    
// // // //   AcY=Wire.read()<<8|Wire.read();  
// // // //   AcZ=Wire.read()<<8|Wire.read();  
// // // //   GyX=Wire.read()<<8|Wire.read();  
// // // //   GyY=Wire.read()<<8|Wire.read();  
// // // //   GyZ=Wire.read()<<8|Wire.read();  
  
// // // //   Serial.print("Accelerometer: ");
// // // //   Serial.print("X = "); Serial.print(AcX);
// // // //   Serial.print(" | Y = "); Serial.print(AcY);
// // // //   Serial.print(" | Z = ");  Serial.println(AcZ); 
  
// // // //   Serial.print("Gyroscope: ");
// // // //   Serial.print("X  = "); Serial.print(GyX);
// // // //   Serial.print(" | Y = "); Serial.print(GyY);
// // // //   Serial.print(" | Z = "); Serial.println(GyZ);
// // // //   Serial.println(" ");
// // // //   delay(1000);
// // // // }
// // // void setup() {
// // //   Serial.begin(9600);
// // //   Wire.begin();
// // //   Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
// // //   Wire.write(0x6B); // PWR_MGMT_1 register
// // //   Wire.write(0); // set to zero (wakes up the MPU-6050)
// // //   Wire.endTransmission(true);
// // // }
// // // void loop() {
// // //   Wire.beginTransmission(MPU_ADDR);
// // //   Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
// // //   Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
// // //   Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
  
// // //   // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
// // //   accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
// // //   accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
// // //   accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
// // //   temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
// // //   gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
// // //   gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
// // //   gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  
// // //   // print out data
// // //   Serial.print("aX = "); Serial.print(convert_int16_to_str(accelerometer_x));
// // //   Serial.print(" | aY = "); Serial.print(convert_int16_to_str(accelerometer_y));
// // //   Serial.print(" | aZ = "); Serial.print(convert_int16_to_str(accelerometer_z));
// // //   // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
// // //   Serial.print(" | tmp = "); Serial.print(temperature/340.00+36.53);
// // //   Serial.print(" | gX = "); Serial.print(convert_int16_to_str(gyro_x));
// // //   Serial.print(" | gY = "); Serial.print(convert_int16_to_str(gyro_y));
// // //   Serial.print(" | gZ = "); Serial.print(convert_int16_to_str(gyro_z));
// // //   Serial.println();
  
// // //   // delay
// // //   delay(1000);
// // // }

// // #include<Wire.h>
// // #include <math.h>
// // const int MPU=0x68;
// // int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
// // double pitch,roll;
// // void getAngle(int16_t x, int16_t y, int16_t z);

// // void setup(){
// // Wire.begin();
// // Wire.beginTransmission(MPU);
// // Wire.write(0x6B);
// // Wire.write(0);
// // Wire.endTransmission(true);
// // Serial.begin(9600);
// // }
// // void loop(){
// // Wire.beginTransmission(MPU);
// // Wire.write(0x3B);
// // Wire.endTransmission(false);
// // Wire.requestFrom(MPU,14,true);

// // int AcXoff,AcYoff,AcZoff,GyXoff,GyYoff,GyZoff;
// // int temp,toff;
// // double t,tx,tf;

// // //Acceleration data correction
// // AcXoff = -950;
// // AcYoff = -300;
// // AcZoff = 0;

// // //Temperature correction
// // toff = -1600;

// // //Gyro correction
// // GyXoff = 480;
// // GyYoff = 170;
// // GyZoff = 210;

// // //read accel data
// // AcX=(Wire.read()<<8|Wire.read()) + AcXoff;
// // AcY=(Wire.read()<<8|Wire.read()) + AcYoff;
// // AcZ=(Wire.read()<<8|Wire.read()) + AcYoff;

// // //read temperature data
// // temp=(Wire.read()<<8|Wire.read()) + toff;
// // tx=temp;
// // t = tx/340 + 36.53;
// // tf = (t * 9/5) + 32;

// // //read gyro data
// // GyX=(Wire.read()<<8|Wire.read()) + GyXoff;
// // GyY=(Wire.read()<<8|Wire.read()) + GyYoff;
// // GyZ=(Wire.read()<<8|Wire.read()) + GyZoff;

// // //get pitch/roll
// // getAngle(AcX, AcY, AcZ);

// // //send the data out the serial port
// // Serial.print("Angle: ");
// // Serial.print("Pitch = "); Serial.print(pitch);
// // Serial.print(" | Roll = "); Serial.println(roll);

// // Serial.print("Temp: ");
// // Serial.print("Temp(F) = "); Serial.print(tf);
// // Serial.print(" | Temp(C) = "); Serial.println(t);

// // Serial.print("Accelerometer: ");
// // Serial.print("X = "); Serial.print(AcX);
// // Serial.print(" | Y = "); Serial.print(AcY);
// // Serial.print(" | Z = "); Serial.println(AcZ);

// // Serial.print("Gyroscope: ");
// // Serial.print("X = "); Serial.print(GyX);
// // Serial.print(" | Y = "); Serial.print(GyY);
// // Serial.print(" | Z = "); Serial.println(GyZ);
// // Serial.println(" ");
// // delay(333);
// // }

// // //convert the accel data to pitch/roll
// // void getAngle(int Vx,int Vy,int Vz) {
// // double x = Vx;
// // double y = Vy;
// // double z = Vz;

// // pitch = atan(x/sqrt((y*y) + (z*z)));
// // roll = atan(y/sqrt((x*x) + (z*z)));
// // //convert radians into degrees
// // pitch = pitch * (180.0/3.14);
// // roll = roll * (180.0/3.14) ;

// // }

// // #include <Adafruit_MPU6050.h>
// // #include <Adafruit_Sensor.h>
// // #include <Wire.h>

// // Adafruit_MPU6050 mpu;

// // void setup(void) {
// // 	Serial.begin(115200);

// // 	// Try to initialize!
// // 	if (!mpu.begin()) {
// // 		Serial.println("Failed to find MPU6050 chip");
// // 		while (1) {
// // 		  delay(10);
// // 		}
// // 	}
// // 	Serial.println("MPU6050 Found!");

// // 	// set accelerometer range to +-8G
// // 	mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

// // 	// set gyro range to +- 500 deg/s
// // 	mpu.setGyroRange(MPU6050_RANGE_500_DEG);

// // 	// set filter bandwidth to 21 Hz
// // 	mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

// // 	delay(100);
// // }

// // void loop() {
// // 	/* Get new sensor events with the readings */
// // 	sensors_event_t a, g, temp;
// // 	mpu.getEvent(&a, &g, &temp);

// // 	/* Print out the values */
// // 	Serial.print("Acceleration X: ");
// // 	Serial.print(a.acceleration.x);
// // 	Serial.print(", Y: ");
// // 	Serial.print(a.acceleration.y);
// // 	Serial.print(", Z: ");
// // 	Serial.print(a.acceleration.z);
// // 	Serial.println(" m/s^2");

// // 	Serial.print("Rotation X: ");
// // 	Serial.print(g.gyro.x);
// // 	Serial.print(", Y: ");
// // 	Serial.print(g.gyro.y);
// // 	Serial.print(", Z: ");
// // 	Serial.print(g.gyro.z);
// // 	Serial.println(" rad/s");

// // 	Serial.print("Temperature: ");
// // 	Serial.print(temp.temperature);
// // 	Serial.println(" degC");

// // 	Serial.println("");
// // 	delay(500);
// // }
// // #include <Adafruit_MPU6050.h>
// // #include <Adafruit_Sensor.h>
// // #include <Wire.h>

// // Adafruit_MPU6050 mpu;

// // const int desiredDegree = 90; // Desired degree to achieve
// // const int tolerance = 2;      // Tolerance range (+/-) for desired degree

// // void setup(void) {
// //   Serial.begin(115200);

// //   // Try to initialize!
// //   if (!mpu.begin()) {
// //     Serial.println("Failed to find MPU6050 chip");
// //     while (1) {
// //       delay(10);
// //     }
// //   }
// //   Serial.println("MPU6050 Found!");

// //   // set accelerometer range to +-8G
// //   mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

// //   // set gyro range to +- 500 deg/s
// //   mpu.setGyroRange(MPU6050_RANGE_500_DEG);

// //   // set filter bandwidth to 21 Hz
// //   mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

// //   delay(100);
// // }

// // bool rotate(int degree) {
// //   sensors_event_t g;
// //   mpu.getEvent(NULL, &g, NULL);

// // //   int gyroX = g.gyro.x;
// //   int gyroZ = g.gyro.z;
// // //   int currentDegree = map(gyroX, -32768, 32767, -90, 90);
// //   int currentDegree = map(gyroZ, -32768, 32767, -90, 90);
// // //   int currentDegree = g.gyro.z;

// //   Serial.print("Current Degree: ");
// //   Serial.println(currentDegree);

// //   // Check if the current degree is within the desired range
// //   if (currentDegree >= (degree - tolerance) && currentDegree <= (degree + tolerance)) {
// //     return true; // The desired degree is reached
// //   } else {
// //     return false; // The desired degree is not reached yet
// //   }
// // }

// // void loop() {
// //   bool degreeReached = false;

// //   while (!degreeReached) {
// //     degreeReached = rotate(desiredDegree);
// //     // Add any other necessary code here
// //   }

// //   // Do something after the desired degree is reached
// //   Serial.println("Desired degree reached!");

// //   delay(500);
// // }

// #include <Wire.h>
// const int MPU = 0x68; // MPU6050 I2C address
// float AccX, AccY, AccZ;
// float GyroX, GyroY, GyroZ;
// float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
// float roll, pitch, yaw;
// float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
// float elapsedTime, currentTime, previousTime;
// int c = 0;

// void calculate_IMU_error();

// void setup() {
//   Serial.begin(19200);
//   Wire.begin();                      // Initialize comunication
//   Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
//   Wire.write(0x6B);                  // Talk to the register 6B
//   Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
//   Wire.endTransmission(true);        //end the transmission
//   /*
//   // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
//   Wire.beginTransmission(MPU);
//   Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
//   Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
//   Wire.endTransmission(true);
//   // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
//   Wire.beginTransmission(MPU);
//   Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
//   Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
//   Wire.endTransmission(true);
//   delay(20);
//   */
//   // Call this function if you need to get the IMU error values for your module
//   calculate_IMU_error();
//   delay(20);
// }

// void loop() {
//   // === Read acceleromter data === //
//   Wire.beginTransmission(MPU);
//   Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
//   Wire.endTransmission(false);
//   Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
//   //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
//   AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
//   AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
//   AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
//   // Calculating Roll and Pitch from the accelerometer data
//   accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
//   accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
//   // === Read gyroscope data === //
//   previousTime = currentTime;        // Previous time is stored before the actual time read
//   currentTime = millis();            // Current time actual time read
//   elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
//   Wire.beginTransmission(MPU);
//   Wire.write(0x43); // Gyro data first register address 0x43
//   Wire.endTransmission(false);
//   Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
//   GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
//   GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
//   GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
//   // Correct the outputs with the calculated error values
//   GyroX = GyroX + 0.56; // GyroErrorX ~(-0.56)
//   GyroY = GyroY - 2; // GyroErrorY ~(2)
//   GyroZ = GyroZ + 0.79; // GyroErrorZ ~ (-0.8)
//   // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
//   gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
//   gyroAngleY = gyroAngleY + GyroY * elapsedTime;
//   yaw =  yaw + GyroZ * elapsedTime;
//   // Complementary filter - combine acceleromter and gyro angle values
//   roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
//   pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  
//   // Print the values on the serial monitor
//   Serial.print(roll);
//   Serial.print("/");
//   Serial.print(pitch);
//   Serial.print("/");
//   Serial.println(yaw);
// }

// void calculate_IMU_error() {
//   // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
//   // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
//   // Read accelerometer values 200 times
//   while (c < 200) {
//     Wire.beginTransmission(MPU);
//     Wire.write(0x3B);
//     Wire.endTransmission(false);
//     Wire.requestFrom(MPU, 6, true);
//     AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
//     AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
//     AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
//     // Sum all readings
//     AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
//     AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
//     c++;
//   }
//   //Divide the sum by 200 to get the error value
//   AccErrorX = AccErrorX / 200;
//   AccErrorY = AccErrorY / 200;
//   c = 0;
//   // Read gyro values 200 times
//   while (c < 200) {
//     Wire.beginTransmission(MPU);
//     Wire.write(0x43);
//     Wire.endTransmission(false);
//     Wire.requestFrom(MPU, 6, true);
//     GyroX = Wire.read() << 8 | Wire.read();
//     GyroY = Wire.read() << 8 | Wire.read();
//     GyroZ = Wire.read() << 8 | Wire.read();
//     // Sum all readings
//     GyroErrorX = GyroErrorX + (GyroX / 131.0);
//     GyroErrorY = GyroErrorY + (GyroY / 131.0);
//     GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
//     c++;
//   }
//   //Divide the sum by 200 to get the error value
//   GyroErrorX = GyroErrorX / 200;
//   GyroErrorY = GyroErrorY / 200;
//   GyroErrorZ = GyroErrorZ / 200;
//   // Print the error values on the Serial Monitor
//   Serial.print("AccErrorX: ");
//   Serial.println(AccErrorX);
//   Serial.print("AccErrorY: ");
//   Serial.println(AccErrorY);
//   Serial.print("GyroErrorX: ");
//   Serial.println(GyroErrorX);
//   Serial.print("GyroErrorY: ");
//   Serial.println(GyroErrorY);
//   Serial.print("GyroErrorZ: ");
//   Serial.println(GyroErrorZ);
// }
