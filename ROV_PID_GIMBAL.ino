// Code for the Trident ROV Gimbal

#include <MadgwickAHRS.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>


// Set MPU6050 object
Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_accel, *mpu_gyro;

// Create Madgwick filter object and set filter rate
const byte filterRate = 210;
Madgwick filter;

// Initialize and set filter rate
const double microsPerReading = 1000000/filterRate;
unsigned long microsPrevious;
unsigned long microsNow;

// Initialize angle variables for servo control
float rollAngle;
float outputAngleMicros;
float initialPosAng;
float Kd;
float Kp;
float rateError;

// Define Water Sensor and Laser Pins
const int water_sensor = 8;
const int laser_1 = 6;
const int laser_2 = 4;
const int servoPin = 9;

// Create Servo Object
Servo myservo;

// Set Calibration swing Count
//const byte swingCount = 2;

// Initialize variables for storing acceleration indices
float accel;
float accelMax;
int ind;
int indMax1;
int indMax2;

// Set Servo Angle Boundaries
const int minPos = 40;                                                // VERIFY
const int maxPos = 122;

// Define Initial Servo Position before calibration
float pos =  minPos; 

// Set up Derivative control
float lastError;



void setup() {
  //Serial.begin(115200);
  //while (!Serial)
    //delay(10); // will pause Zero, Leonardo, etc until serial console opens

  // Try to initialize!
  if (!mpu.begin()) {
 //   Serial.println("Failed to find MPU6050 chip"); // For debugging
    while (1) {
      delay(10);
    }
  }
//  Serial.println("MPU6050 Found!"); // For debugging

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G); // Set accelerometer range: 2, 4, 8, or 16 G 
  
  mpu.setGyroRange(MPU6050_RANGE_250_DEG); // Set gyroscope range: 250, 500, 1000 or 2000 deg/s

  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ); // Set bandwidth of filter: 260, 184, 94, 44, 21, 10, or 5 Hz
    
  // Setup pins used for water detector sensor and warning light
  pinMode(water_sensor, INPUT);
  pinMode(laser_1, OUTPUT);

  // Setup Servo
  myservo.attach(9, 1000, 2000);
  delay(1000);
  myservo.write(pos);
  delay(20000); // Wait 20 seconds to attach and assemble the gimbal
/*
  // Run Servo and Attitude calibration function and store indices
  int indSum = 0;
  
  for (int i = 1; i <= swingCount; i++) {
    calibration();
    indSum = indSum + indMax1 + indMax2;
    //indSum = indSum +indMax1;
  }

  // Take average of the indices to find the angle associated with level initial position
  float indAvg = round(indSum/(2*swingCount));
  //float indAvg = indSum/swingCount; 
  pos = minPos + indAvg;
  initialPosAng = pos;
  //Serial.print("Initial Position: ");
  //Serial.println(pos);
*/
  pos = 80;
  // Set Servo to initial position and set inputAngle
  myservo.write(pos);
  delay(1000);

  // Start the Madgwick Filter    
  filter.begin(filterRate);
  
  // Initialize loop time counter
  microsPrevious = micros();
  
}


void loop() {
  
  // Get new sensor events with the readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  microsNow = micros();
  //Serial.print("Loop Time: ");
  float microsLoop = microsNow-microsPrevious;
  //Serial.println(microsLoop);
  //Serial.print(microsPrevious); Serial.print(", ");

  // Leak Detection and temperature warning 
  if ((temp.temperature < 80) && (digitalRead(water_sensor) == LOW)) {

    digitalWrite(laser_1, LOW); // No warning light
    
    //if (microsLoop >= microsPerReading) {
      float gx = g.gyro.x*57.2957795131; // convert from rad/s to deg/s
      float gy = g.gyro.y*57.2957795131;
      float gz = g.gyro.z*57.2957795131;
      filter.updateIMU(gx, gy, gz, a.acceleration.x, a.acceleration.y, a.acceleration.z);

      rollAngle = filter.getPitch(); // Orientation of the IMU
      rateError = (rollAngle-lastError)/microsLoop;
      Kp = 0.09;                                                 // TUNE
      Kd = -100;
      lastError = rollAngle;
      
      //Serial.print("Roll Angle: ");
      //Serial.print(rollAngle); Serial.print(", ");

      //Serial.print("Correction Angle: ");
      //Serial.print(Kp*rollAngle + Kd*rateError); Serial.print(", ");


      //Serial.print(a.acceleration.x); Serial.print(", ");
      //Serial.print(a.acceleration.y); Serial.print(", ");
      //Serial.print(a.acceleration.z); Serial.print(", ");
      //Serial.print(gx); Serial.print(", ");
      //Serial.print(gy); Serial.print(", ");
      //Serial.print(gz); Serial.print(", ");
      
      pos = pos - (Kp*rollAngle + Kd*rateError);                           
      //Serial.print("Position: ");
      //Serial.println(pos);
      //Serial.println(" ");
      
      if (pos > (maxPos)) {                   // put angle within physical limits of the enclosure 
        pos = maxPos;
        outputAngleMicros = map(pos, 0, 180, 1000, 2000); // Map output angle to microseconds for better precision
        myservo.writeMicroseconds(outputAngleMicros);                 // Write servo output
        //Serial.println(pos);
        //delay(1);
      }
      else if (pos < (minPos)){               // put angle within physical limits of the enclosure
        pos = minPos;
        outputAngleMicros = map(pos, 0, 180, 1000, 2000); // Map output angle to microseconds for better precision
        myservo.writeMicroseconds(outputAngleMicros);                 // Write servo output
        //Serial.println(pos);
        //delay(1);
      }
      else {
        outputAngleMicros = map(pos, 0, 180, 1000, 2000); // Map output angle to microseconds for better precision
        myservo.writeMicroseconds(outputAngleMicros);                 // Write servo output
        //Serial.println(pos);
        //delay(1);
      }
         
    //}
  }
  else {
    digitalWrite(laser_1, HIGH); // warning lights flash
    delay(500);
    digitalWrite(laser_1, LOW);
    delay(500);
  }


  /* Print out the values for debugging only */
  /*Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");*/

//  Serial.print("Temperature: ");
//  Serial.print(temp.temperature);
//  Serial.println(" degC");

//  Serial.println("");
//  delay(500);

  // Leak Detection and Warning
  if(digitalRead(water_sensor) == LOW) {
    digitalWrite(laser_1, LOW);
  }
  else {
    digitalWrite(laser_1, HIGH);
    delay(500);
    digitalWrite(laser_1, LOW);
    delay(500);
  }

  // increment loop time
  microsPrevious = microsNow;

}


// Run a full sweep to find and store the indices of the maximum acceleration as the servo sweeps back and forth
int calibration() {
 
  sensors_event_t a;
  
  accelMax = 0; // intialize comparison value to isolate max acceleration
  ind = 0;      // initialize indexing value
  
  for (pos = minPos; pos <= maxPos; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    mpu_accel = mpu.getAccelerometerSensor(); // Get accelerometer data
    mpu_accel->getEvent(&a);
    
    myservo.write(pos);                   // tell servo to go to position in variable 'pos'
    delay(8);                            // waits 5 ms for the servo to reach the position

    accel = abs(a.acceleration.y);        // isolate y data
    accelMax = max(accel, accelMax);      // compare new and old acceleration to get max index of the sweep

    // store max index
    if (accel == accelMax) {
      indMax1 = ind;
    }   
    ind += 1;                             // increase index counter by 1
  }

  accelMax = 0; // reintialize comparison value to isolate max acceleration
  ind = 0;      // reinitialize indexing value
  
  for (pos = maxPos; pos >= minPos; pos -= 1) { // goes from 180 degrees to 0 degrees
    
    mpu_accel = mpu.getAccelerometerSensor();   // Get accelerometer data
    mpu_accel->getEvent(&a);
    
    myservo.write(pos);                // tell servo to go to position in variable 'pos'
    delay(5);                         // waits 5 ms for the servo to reach the position
    
    accel = a.acceleration.y;          // isolate y data
    accelMax = max(accel, accelMax);   // compare new and old acceleration to get max index of the sweep

    // store max index
    if (accel == accelMax) {
      indMax2 = ind;
    }   
    ind += 1;                          // increase index counter by 1
  }

}
