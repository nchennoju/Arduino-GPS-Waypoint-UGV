#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "C:\Users\nchen\Documents\PlatformIO\Arduino-GPS-Waypoint-UGV\Navigation_Controller\.pio\libdeps\nanoatmega328new\PID\PID_v1.h"
#include "C:\Users\nchen\Documents\PlatformIO\Arduino-GPS-Waypoint-UGV\Navigation_Controller\.pio\libdeps\nanoatmega328new\Servo\src\Servo.h"

#define Length 10
#define SERVO_PIN 9
#define ESC_PIN 6
#define WAYPOINT_THRESHOLD_STOP 3
#define WAYPOINT_THRESHOLD_SLOW 10
#define STOP 1500
#define SLOW 1400
#define FAST 1400

int getAngle(char in[], int length);
int getDistance(char in[], int length);
void setSteering(int val);

void read_mpu_6050_data();
void calibrate_gyro();
void config_gyro();
void read_magnetometer();
void calibrate_magnetometer();
void configure_magnetometer();

// -----------
// COMPASS START
// -----------

// ----- Gyro
#define MPU9250_I2C_address 0x68                                        // I2C address for MPU9250 
#define MPU9250_I2C_master_enable 0x6A                                  // USER_CTRL[5] = I2C_MST_EN
#define MPU9250_Interface_bypass_mux_enable 0x37                        // INT_PIN_CFG[1]= BYPASS_EN

#define Frequency 125                                                   // 8mS sample interval 
#define Sensitivity 65.5                                                // Gyro sensitivity (see data sheet)

#define Sensor_to_deg 1/(Sensitivity*Frequency)                         // Convert sensor reading to degrees
#define Sensor_to_rad Sensor_to_deg*DEG_TO_RAD                          // Convert sensor reading to radians

#define Loop_time 1000000/Frequency                                     // Loop time (uS)
long    Loop_start;                                                     // Loop start time (uS)

int     Gyro_x,     Gyro_y,     Gyro_z;
long    Gyro_x_cal, Gyro_y_cal, Gyro_z_cal;
float   Gyro_pitch, Gyro_roll, Gyro_yaw;
float   Gyro_pitch_output, Gyro_roll_output;

// ----- Accelerometer
long    Accel_x,      Accel_y,      Accel_z,    Accel_total_vector;
float   Accel_pitch,  Accel_roll;

// ----- Magnetometer
#define AK8963_I2C_address 0x0C                                             // I2C address for AK8963
#define AK8963_cntrl_reg_1 0x0A                                             // CNTL[4]=#bits, CNTL[3:0]=mode
#define AK8963_status_reg_1 0x02                                            // ST1[0]=data ready
#define AK8963_data_ready_mask 0b00000001                                   // Data ready mask
#define AK8963_overflow_mask 0b00001000                                     // Magnetic sensor overflow mask
#define AK8963_data 0x03                                                    // Start address of XYZ data                                                                
#define AK8963_fuse_ROM 0x10                                                // X,Y,Z fuse ROM

// ----- Compass heading
/*
  The magnetic declination for Lower Hutt, New Zealand is +22.5833 degrees
  Obtain your magnetic declination from http://www.magnetic-declination.com/
  Uncomment the declination code within the main loop() if you want True North.
*/
float   Declination = +15;                                             //  Degrees ... replace this declination with yours
int     Heading;

int     Mag_x,                Mag_y,                Mag_z;                  // Raw magnetometer readings
float   Mag_x_dampened,       Mag_y_dampened,       Mag_z_dampened;
float   Mag_x_hor, Mag_y_hor;
float   Mag_pitch, Mag_roll;

// ----- Record compass offsets, scale factors, & ASA values
/*
   These values seldom change ... an occasional check is sufficient
   (1) Open your Arduino "Serial Monitor
   (2) Set "Record_data=true;" then upload & run program.
   (3) Replace the values below with the values that appear on the Serial Monitor.
   (4) Set "Record_data = false;" then upload & rerun program.
*/
bool    Record_data = false;
// For Rover
int     Mag_x_offset = 732,      Mag_y_offset = -72,     Mag_z_offset = -40;   // Hard-iron offsets
float   Mag_x_scale = 1.01,     Mag_y_scale = 1.07,     Mag_z_scale = 0.93;    // Soft-iron scale factors
float   ASAX = 1.18,            ASAY = 1.20,            ASAZ = 1.14;           // (A)sahi (S)ensitivity (A)djustment fuse ROM values.

// ----- LED
const int LED = 13;                     // Status LED

// ----- Flags
bool Gyro_synchronised = false;
bool Flag = false;

// ----- Debug
#define Switch A0                       // Connect an SPST switch between A0 and GND to enable/disable tilt stabilazation
long Loop_start_time;
long Debug_start_time;

// -----------
// COMPASS END
// -----------

// SERIAL COMMUNICATION
SoftwareSerial COM(A1, A2);
char text[Length];
int i = 0;

// ACTUATOR INITIALIZATION
Servo steering;
Servo esc;

int thetaScaled, angle, distance;

// PID
double Setpoint, Input, Output;
double Kp=2.25, Ki=0.7, Kd=1.8;
//double Kp=3.4, Ki=0.7, Kd=4.0;
double Kp2=2.25, Ki2=0.7, Kd2=1.8;
PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);



void setup() {
  Serial.begin(9600);                                //Use only for debugging
  COM.begin(9600);

  Wire.begin();                                         //Start I2C as master
  Wire.setClock(400000);

  // ----- Provision to disable tilt stabilization
  /*
     Connect a jumper wire between A0 and GRN to disable the "tilt stabilazation"
  */
  pinMode(Switch, INPUT_PULLUP);                        // Short A0 to GND to disable tilt stabilization

  // ----- Status LED
  pinMode(LED, OUTPUT);                                 // Set LED (pin 13) as output
  digitalWrite(LED, LOW);                               // Turn LED off


  // ----- Configure the magnetometer
  configure_magnetometer();

  // ----- Calibrate the magnetometer
  /*
     Calibrate only needs to be done occasionally.
     Enter the magnetometer values into the "header"
     then set "Record_data = false".
  */
  if (Record_data == true)
  {
    calibrate_magnetometer();
  }

  // ----- Configure the gyro & magnetometer
  config_gyro();

  calibrate_gyro();

  // ----- Display "heading, pitch, roll" headings

  Debug_start_time = micros();                          // Controls data display rate to Serial Monitor
  Loop_start_time = micros();                           // Controls the Gyro refresh rate

  Serial.println("SETUP done");


  // CONTROLS
  steering.attach(SERVO_PIN);
  setSteering(0);
  //esc.attach(ESC_PIN);
  //esc.writeMicroseconds(1500);
  //delay(2000);

  // INDICATE STARTUP
  /*esc.writeMicroseconds(1600);
  delay(500);
  esc.writeMicroseconds(1400);
  delay(500);*/
  
  // PID Setup
  Input = 0;
  Setpoint = 0;
  pid.SetOutputLimits(-100, 100);
  pid.SetMode(AUTOMATIC);

}

void loop() {

  // GET GPS DATA
  COM.listen();
  while (COM.available() > 0) {
    text[i] = COM.read();
    Serial.print(text[i]);
    i+=1;
    //delay(5);
  }
  if(i != 0){
    /*int tmpAngle = getAngle(text, Length);
    if(tmpAngle <= 360 && tmpAngle >= 0){
      angle = tmpAngle;
    }
    int tmpDistance = getDistance(text, Length);
    if(tmpDistance >= 0 && tmpDistance <= 500) {
      distance = tmpDistance;
    }*/
    angle = getAngle(text, Length);
    distance = getDistance(text, Length);
  }
  i = 0;

    ////////////////////////////////////////////
  //        PITCH & ROLL CALCULATIONS       //
  ////////////////////////////////////////////

  /*
     --------------------
     MPU-9250 Orientation
     --------------------
     Component side up
     X-axis facing forward
  */

  // ----- read the raw accelerometer and gyro data
  read_mpu_6050_data();                                             // Read the raw acc and gyro data from the MPU-6050

  // ----- Adjust for offsets
  Gyro_x -= Gyro_x_cal;                                             // Subtract the offset from the raw gyro_x value
  Gyro_y -= Gyro_y_cal;                                             // Subtract the offset from the raw gyro_y value
  Gyro_z -= Gyro_z_cal;                                             // Subtract the offset from the raw gyro_z value

  // ----- Calculate travelled angles
  /*
    ---------------------------
    Adjust Gyro_xyz signs for:
    ---------------------------
    Pitch (Nose - up) = +ve reading
    Roll (Right - wing down) = +ve reading
    Yaw (Clock - wise rotation)  = +ve reading
  */
  Gyro_pitch += -Gyro_y * Sensor_to_deg;                            // Integrate the raw Gyro_y readings
  Gyro_roll += Gyro_x * Sensor_to_deg;                              // Integrate the raw Gyro_x readings
  Gyro_yaw += -Gyro_z * Sensor_to_deg;                              // Integrate the raw Gyro_x readings

  // ----- Compensate pitch and roll for gyro yaw
  Gyro_pitch += Gyro_roll * sin(Gyro_z * Sensor_to_rad);            // Transfer the roll angle to the pitch angle if the Z-axis has yawed
  Gyro_roll -= Gyro_pitch * sin(Gyro_z * Sensor_to_rad);            // Transfer the pitch angle to the roll angle if the Z-axis has yawed

  // ----- Accelerometer angle calculations
  Accel_total_vector = sqrt((Accel_x * Accel_x) + (Accel_y * Accel_y) + (Accel_z * Accel_z));   // Calculate the total (3D) vector
  Accel_pitch = asin((float)Accel_x / Accel_total_vector) * RAD_TO_DEG;                         //Calculate the pitch angle
  Accel_roll = asin((float)Accel_y / Accel_total_vector) * RAD_TO_DEG;                         //Calculate the roll angle

  // ----- Zero any residual accelerometer readings
  /*
     Place the accelerometer on a level surface
     Adjust the following two values until the pitch and roll readings are zero
  */
  Accel_pitch -= -0.2f;                                             //Accelerometer calibration value for pitch
  Accel_roll -= 1.1f;                                               //Accelerometer calibration value for roll

  // ----- Correct for any gyro drift
  if (Gyro_synchronised)
  {
    // ----- Gyro & accel have been synchronised
    Gyro_pitch = Gyro_pitch * 0.9996 + Accel_pitch * 0.0004;        //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    Gyro_roll = Gyro_roll * 0.9996 + Accel_roll * 0.0004;           //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else
  {
    // ----- Synchronise gyro & accel
    Gyro_pitch = Accel_pitch;                                       //Set the gyro pitch angle equal to the accelerometer pitch angle
    Gyro_roll = Accel_roll;                                         //Set the gyro roll angle equal to the accelerometer roll angle
    Gyro_synchronised = true;                                             //Set the IMU started flag
  }

  // ----- Dampen the pitch and roll angles
  Gyro_pitch_output = Gyro_pitch_output * 0.9 + Gyro_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  Gyro_roll_output = Gyro_roll_output * 0.9 + Gyro_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value

  ////////////////////////////////////////////
  //        MAGNETOMETER CALCULATIONS       //
  ////////////////////////////////////////////
  /*
     --------------------------------
     Instructions for first time use
     --------------------------------
     Calibrate the compass for Hard-iron and Soft-iron
     distortion by temporarily setting the header to read
     bool    Record_data = true;

     Turn on your Serial Monitor before uploading the code.

     Slowly tumble the compass in all directions until a
     set of readings appears in the Serial Monitor.

     Copy these values into the appropriate header locations.

     Edit the header to read
     bool    Record_data = false;

     Upload the above code changes to your Arduino.

     This step only needs to be done occasionally as the
     values are reasonably stable.
  */

  // ----- Read the magnetometer
  read_magnetometer();

  // ----- Fix the pitch, roll, & signs
  /*
     MPU-9250 gyro and AK8963 magnetometer XY axes are orientated 90 degrees to each other
     which means that Mag_pitch equates to the Gyro_roll and Mag_roll equates to the Gryro_pitch

     The MPU-9520 and AK8963 Z axes point in opposite directions
     which means that the sign for Mag_pitch must be negative to compensate.
  */
  Mag_pitch = -Gyro_roll_output * DEG_TO_RAD;
  Mag_roll = Gyro_pitch_output * DEG_TO_RAD;

  // ----- Apply the standard tilt formulas
  Mag_x_hor = Mag_x * cos(Mag_pitch) + Mag_y * sin(Mag_roll) * sin(Mag_pitch) - Mag_z * cos(Mag_roll) * sin(Mag_pitch);
  Mag_y_hor = Mag_y * cos(Mag_roll) + Mag_z * sin(Mag_roll);

  // ----- Disable tilt stabization if switch closed
  if (!(digitalRead(Switch)))
  {
    // ---- Test equations
    Mag_x_hor = Mag_x;
    Mag_y_hor = Mag_y;
  }

  // ----- Dampen any data fluctuations
  Mag_x_dampened = Mag_x_dampened * 0.9 + Mag_x_hor * 0.1;
  Mag_y_dampened = Mag_y_dampened * 0.9 + Mag_y_hor * 0.1;

  // ----- Calculate the heading
  Heading = atan2(Mag_x_dampened, Mag_y_dampened) * RAD_TO_DEG;  // Magnetic North

  /*
     By convention, declination is positive when magnetic north
     is east of true north, and negative when it is to the west.
  */

  Heading += Declination;               // Geographic North
  if (Heading > 360.0) Heading -= 360.0;
  if (Heading < 0.0) Heading += 360.0;

  // ----- Allow for under/overflow
  if (Heading < 0) Heading += 360;
  if (Heading >= 360) Heading -= 360;


  
  

  //TURNING CONTROL: Heading (current heading), angle (required heading)
  if(Heading > 180 && angle <= 180) {
    thetaScaled = Heading - (angle + 360);
  } else if(angle > 180 && Heading <= 180) {
    thetaScaled = (Heading + 360) - angle;
  } else {
    thetaScaled = Heading - angle;
  }
  thetaScaled *= (100.0/180.0);
  thetaScaled = -thetaScaled;


  /*if(abs(thetaScaled) > 15) {
    pid.SetTunings(Kp, Ki, Kd);
  }else{
    pid.SetTunings(Kp2, Ki2, Kd2);
  }*/

  // STEERING CONTROL
  Input = thetaScaled;
  pid.Compute();
  setSteering(Output);

  Serial.print("\t\t\tTheta: " + String(thetaScaled) + "\tAngle: " + String(angle) + "\tDistance: " + String(distance));
  
  Serial.println();


}

void setSteering(int val) {
  // steering center at 95
  // range [70, 120]
  // input parameter val is a value [-100, 100], with negative values turning left
  val = (val < -100) ? -100: val;
  val = (val > 100) ? 100: val;
  int pos = (val/4.0) + 95;
  steering.write(pos);
}

int getAngle(char in[], int length) {
    int j = 0;
    String temp = "";
    
    while(in[j] != '\t' && in[j] != '\n' && in[j] != ' ' && j < length) {
        temp += in[j];
        j++;
    }

    return temp.toInt();
}

int getDistance(char in[], int length) {
    int k = 0;

    while(in[k] != '\t' && in[k] != ' ' && in[k] != '\n' && k < length) {
      k++;
    }

    String temp = "";
    while (k < length) {
        temp += in[k];
        k++;
    }
    return temp.toInt();
}

void configure_magnetometer()
{
  /*
     The MPU-9250 contains an AK8963 magnetometer and an
     MPU-6050 gyro/accelerometer within the same package.

     To access the AK8963 magnetometer chip The MPU-9250 I2C bus
     must be changed to pass-though mode. To do this we must:
      - disable the MPU-9250 slave I2C and
      - enable the MPU-9250 interface bypass mux
  */
  // ----- Disable MPU9250 I2C master interface
  Wire.beginTransmission(MPU9250_I2C_address);                      // Open session with MPU9250
  Wire.write(MPU9250_I2C_master_enable);                            // Point USER_CTRL[5] = I2C_MST_EN
  Wire.write(0x00);                                                 // Disable the I2C master interface
  Wire.endTransmission();

  // ----- Enable MPU9250 interface bypass mux
  Wire.beginTransmission(MPU9250_I2C_address);                      // Open session with MPU9250
  Wire.write(MPU9250_Interface_bypass_mux_enable);                  // Point to INT_PIN_CFG[1] = BYPASS_EN
  Wire.write(0x02);                                                 // Enable the bypass mux
  Wire.endTransmission();

  // ----- Access AK8963 fuse ROM
  /* The factory sensitivity readings for the XYZ axes are stored in a fuse ROM.
     To access this data we must change the AK9863 operating mode.
  */
  Wire.beginTransmission(AK8963_I2C_address);                       // Open session with AK8963
  Wire.write(AK8963_cntrl_reg_1);                                   // CNTL[3:0] mode bits
  Wire.write(0b00011111);                                           // Output data=16-bits; Access fuse ROM
  Wire.endTransmission();
  delay(100);                                                       // Wait for mode change

  // ----- Get factory XYZ sensitivity adjustment values from fuse ROM
  /* There is a formula on page 53 of "MPU-9250, Register Map and Decriptions, Revision 1.4":
      Hadj = H*(((ASA-128)*0.5)/128)+1 where
      H    = measurement data output from data register
      ASA  = sensitivity adjustment value (from fuse ROM)
      Hadj = adjusted measurement data (after applying
  */
  Wire.beginTransmission(AK8963_I2C_address);                       // Open session with AK8963
  Wire.write(AK8963_fuse_ROM);                                      // Point to AK8963 fuse ROM
  Wire.endTransmission();
  Wire.requestFrom(AK8963_I2C_address, 3);                          // Request 3 bytes of data
  while (Wire.available() < 3);                                     // Wait for the data
  ASAX = (Wire.read() - 128) * 0.5 / 128 + 1;                       // Adjust data
  ASAY = (Wire.read() - 128) * 0.5 / 128 + 1;
  ASAZ = (Wire.read() - 128) * 0.5 / 128 + 1;

  // ----- Power down AK8963 while the mode is changed
  /*
     This wasn't necessary for the first mode change as the chip was already powered down
  */
  Wire.beginTransmission(AK8963_I2C_address);                       // Open session with AK8963
  Wire.write(AK8963_cntrl_reg_1);                                   // Point to mode control register
  Wire.write(0b00000000);                                           // Set mode to power down
  Wire.endTransmission();
  delay(100);                                                       // Wait for mode change

  // ----- Set output to mode 2 (16-bit, 100Hz continuous)
  Wire.beginTransmission(AK8963_I2C_address);                       // Open session with AK8963
  Wire.write(AK8963_cntrl_reg_1);                                   // Point to mode control register
  Wire.write(0b00010110);                                           // Output=16-bits; Measurements = 100Hz continuous
  Wire.endTransmission();
  delay(100);                                                       // Wait for mode change
}

// -------------------------------
//  Calibrate magnetometer
// -------------------------------
void calibrate_magnetometer()
{
  // ----- Locals
  int mag_x, mag_y, mag_z;
  int status_reg_2;                                               // ST2 status register

  int mag_x_min =  32767;                                         // Raw data extremes
  int mag_y_min =  32767;
  int mag_z_min =  32767;
  int mag_x_max = -32768;
  int mag_y_max = -32768;
  int mag_z_max = -32768;

  float chord_x,  chord_y,  chord_z;                              // Used for calculating scale factors
  float chord_average;

  // ----- Record min/max XYZ compass readings
  for (int counter = 0; counter < 16000 ; counter ++)             // Run this code 16000 times
  {
    Loop_start = micros();                                        // Start loop timer
    //if (counter % 1000 == 0)lcd.print(".");                        // Print a dot on the LCD every 1000 readings

    // ----- Point to status register 1
    Wire.beginTransmission(AK8963_I2C_address);                   // Open session with AK8963
    Wire.write(AK8963_status_reg_1);                              // Point to ST1[0] status bit
    Wire.endTransmission();
    Wire.requestFrom(AK8963_I2C_address, 1);                      // Request 1 data byte
    while (Wire.available() < 1);                                 // Wait for the data
    if (Wire.read() & AK8963_data_ready_mask)                     // Check data ready bit
    {
      // ----- Read data from each axis (LSB,MSB)
      Wire.requestFrom(AK8963_I2C_address, 7);                    // Request 7 data bytes
      while (Wire.available() < 7);                               // Wait for the data
      mag_x = (Wire.read() | Wire.read() << 8) * ASAX;            // Combine LSB,MSB X-axis, apply ASA corrections
      mag_y = (Wire.read() | Wire.read() << 8) * ASAY;            // Combine LSB,MSB Y-axis, apply ASA corrections
      mag_z = (Wire.read() | Wire.read() << 8) * ASAZ;            // Combine LSB,MSB Z-axis, apply ASA corrections
      status_reg_2 = Wire.read();                                 // Read status and signal data read

      // ----- Validate data
      if (!(status_reg_2 & AK8963_overflow_mask))                 // Check HOFL flag in ST2[3]
      {
        // ----- Find max/min xyz values
        mag_x_min = min(mag_x, mag_x_min);
        mag_x_max = max(mag_x, mag_x_max);
        mag_y_min = min(mag_y, mag_y_min);
        mag_y_max = max(mag_y, mag_y_max);
        mag_z_min = min(mag_z, mag_z_min);
        mag_z_max = max(mag_z, mag_z_max);
      }
    }
    delay(4);                                                     // Time interval between magnetometer readings
  }

  // ----- Calculate hard-iron offsets
  Mag_x_offset = (mag_x_max + mag_x_min) / 2;                     // Get average magnetic bias in counts
  Mag_y_offset = (mag_y_max + mag_y_min) / 2;
  Mag_z_offset = (mag_z_max + mag_z_min) / 2;

  // ----- Calculate soft-iron scale factors
  chord_x = ((float)(mag_x_max - mag_x_min)) / 2;                 // Get average max chord length in counts
  chord_y = ((float)(mag_y_max - mag_y_min)) / 2;
  chord_z = ((float)(mag_z_max - mag_z_min)) / 2;

  chord_average = (chord_x + chord_y + chord_z) / 3;              // Calculate average chord length

  Mag_x_scale = chord_average / chord_x;                          // Calculate X scale factor
  Mag_y_scale = chord_average / chord_y;                          // Calculate Y scale factor
  Mag_z_scale = chord_average / chord_z;                          // Calculate Z scale factor

  // ----- Record magnetometer offsets
  
  //   When active this feature sends the magnetometer data
  //   to the Serial Monitor then halts the program.
  
  if (Record_data == true)
  {
    // ----- Display data extremes
    Serial.print("XYZ Max/Min: ");
    Serial.print(mag_x_min); Serial.print("\t");
    Serial.print(mag_x_max); Serial.print("\t");
    Serial.print(mag_y_min); Serial.print("\t");
    Serial.print(mag_y_max); Serial.print("\t");
    Serial.print(mag_z_min); Serial.print("\t");
    Serial.println(mag_z_max);
    Serial.println("");

    // ----- Display hard-iron offsets
    Serial.print("Hard-iron: ");
    Serial.print(Mag_x_offset); Serial.print("\t");
    Serial.print(Mag_y_offset); Serial.print("\t");
    Serial.println(Mag_z_offset);
    Serial.println("");

    // ----- Display soft-iron scale factors
    Serial.print("Soft-iron: ");
    Serial.print(Mag_x_scale); Serial.print("\t");
    Serial.print(Mag_y_scale); Serial.print("\t");
    Serial.println(Mag_z_scale);
    Serial.println("");

    // ----- Display fuse ROM values
    Serial.print("ASA: ");
    Serial.print(ASAX); Serial.print("\t");
    Serial.print(ASAY); Serial.print("\t");
    Serial.println(ASAZ);

    // ----- Halt program
    while (true);                                       // Wheelspin ... program halt
  }
}

// -------------------------------
//  Read magnetometer
// -------------------------------
void read_magnetometer()
{
  // ----- Locals
  int mag_x, mag_y, mag_z;
  int status_reg_2;

  // ----- Point to status register 1
  Wire.beginTransmission(AK8963_I2C_address);                   // Open session with AK8963
  Wire.write(AK8963_status_reg_1);                              // Point to ST1[0] status bit
  Wire.endTransmission();
  Wire.requestFrom(AK8963_I2C_address, 1);                      // Request 1 data byte
  while (Wire.available() < 1);                                 // Wait for the data
  if (Wire.read() & AK8963_data_ready_mask)                     // Check data ready bit
  {
    // ----- Read data from each axis (LSB,MSB)
    Wire.requestFrom(AK8963_I2C_address, 7);                    // Request 7 data bytes
    while (Wire.available() < 7);                               // Wait for the data
    mag_x = (Wire.read() | Wire.read() << 8) * ASAX;            // Combine LSB,MSB X-axis, apply ASA corrections
    mag_y = (Wire.read() | Wire.read() << 8) * ASAY;            // Combine LSB,MSB Y-axis, apply ASA corrections
    mag_z = (Wire.read() | Wire.read() << 8) * ASAZ;            // Combine LSB,MSB Z-axis, apply ASA corrections
    status_reg_2 = Wire.read();                                 // Read status and signal data read

    // ----- Validate data
    if (!(status_reg_2 & AK8963_overflow_mask))                 // Check HOFL flag in ST2[3]
    {
      Mag_x = (mag_x - Mag_x_offset) * Mag_x_scale;
      Mag_y = (mag_y - Mag_y_offset) * Mag_y_scale;
      Mag_z = (mag_z - Mag_z_offset) * Mag_z_scale;
    }
  }
}
// -----------------------------------
//  Configure the gyro & accelerometer
// -----------------------------------
void config_gyro()
{
  // ----- Activate the MPU-6050
  Wire.beginTransmission(0x68);                         //Open session with the MPU-6050
  Wire.write(0x6B);                                     //Point to power management register
  Wire.write(0x00);                                     //Use internal 20MHz clock
  Wire.endTransmission();                               //End the transmission

  // ----- Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                         //Open session with the MPU-6050
  Wire.write(0x1C);                                     //Point to accelerometer configuration reg
  Wire.write(0x10);                                     //Select +/-8g full-scale
  Wire.endTransmission();                               //End the transmission

  // ----- Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                         //Open session with the MPU-6050
  Wire.write(0x1B);                                     //Point to gyroscope configuration
  Wire.write(0x08);                                     //Select 500dps full-scale
  Wire.endTransmission();                               //End the transmission
}

// -----------------------------------
//  Calibrate gyro
// -----------------------------------
void calibrate_gyro()
{


  // ----- Calibrate gyro
  for (int counter = 0; counter < 2000 ; counter ++)    //Run this code 2000 times
  {
    Loop_start = micros();
    read_mpu_6050_data();                               //Read the raw acc and gyro data from the MPU-6050
    Gyro_x_cal += Gyro_x;                               //Add the gyro x-axis offset to the gyro_x_cal variable
    Gyro_y_cal += Gyro_y;                               //Add the gyro y-axis offset to the gyro_y_cal variable
    Gyro_z_cal += Gyro_z;                               //Add the gyro z-axis offset to the gyro_z_cal variable
    while (micros() - Loop_start < Loop_time);           // Wait until "Loop_time" microseconds have elapsed
  }
  Gyro_x_cal /= 2000;                                   //Divide the gyro_x_cal variable by 2000 to get the average offset
  Gyro_y_cal /= 2000;                                   //Divide the gyro_y_cal variable by 2000 to get the average offset
  Gyro_z_cal /= 2000;                                   //Divide the gyro_z_cal variable by 2000 to get the average offset

}

// --------------------
//  Read MPU 6050 data
// --------------------
void read_mpu_6050_data()
{
  // ----- Locals
  int     temperature;                                  // Needed when reading the MPU-6050 data ... not used

  // ----- Point to data
  Wire.beginTransmission(0x68);                         // Start communicating with the MPU-6050
  Wire.write(0x3B);                                     // Point to start of data
  Wire.endTransmission();                               // End the transmission

  // ----- Read the data
  Wire.requestFrom(0x68, 14);                           // Request 14 bytes from the MPU-6050
  while (Wire.available() < 14);                        // Wait until all the bytes are received
  Accel_x = Wire.read() << 8 | Wire.read();             // Combine MSB,LSB Accel_x variable
  Accel_y = Wire.read() << 8 | Wire.read();             // Combine MSB,LSB Accel_y variable
  Accel_z = Wire.read() << 8 | Wire.read();             // Combine MSB,LSB Accel_z variable
  Wire.read() << 8 | Wire.read();         // Combine MSB,LSB temperature variable
  Gyro_x = Wire.read() << 8 | Wire.read();              // Combine MSB,LSB Gyro_x variable
  Gyro_y = Wire.read() << 8 | Wire.read();              // Combine MSB,LSB Gyro_x variable
  Gyro_z = Wire.read() << 8 | Wire.read();              // Combine MSB,LSB Gyro_x variable
}