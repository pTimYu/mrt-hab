/* 
  HAB FC code
  2025-01-11
  By Yusen Ma

  Hardware properties and definition

  SER 1   | 0   MCU     | 5V
  SER 2   | 1           | GND
  SPI CS  | 2           | 3.3V
  BUZZER  | 3        10 | MOSI
  SDA     | 4        9  | MISO
  SCL     | 5        8  | SCK
  TX      | 6        7  | RX
*/

#define TEST_MODE 1
#define ENABLE_BUZZER 1

#define LOCAL_SEA_LEVEL_PRESSURE 101600                   //Local sea level pressure in Pascals, used for altitude calculation
#define TAKEOFF_THRESHOLD 0.6                             //Take-off threshold velocity, used for status update
#define LANDING_THRESHOLD 20

//Test mode parameters
#if TEST_MODE
#define MAX_FLIGHT_TIME 60000                            //Maximum flight time in ms， 1 min = 60000 ms
#define MAX_FLIGHT_HEIGHT 10                             //Desired max flight height in meters
#pragma message("TEST MODE ENABLED")                     //Debug message

#else
#define MAX_FLIGHT_TIME 2100000                           //Maximum flight time in ms， 35 min = 2100000 ms
#define MAX_FLIGHT_HEIGHT 5600                            //Desired max flight height in meters
#pragma message("TEST MODE DISABLED")                     //Debug message
#endif

//IMPORTANT NOTICE
//In this board (Seeed nRF52840 Sense) the commands gave to LED must be inverted, i.e. LED will be switched off if given HIGH command

//Use const int for variable type check to prevent "unforseenable issues"
const int ACTUATOR_PIN_1 = 0;
const int ACTUATOR_PIN_2 = 1;
const int TF_CHIP_SELECT_PIN = 2;
const int BUZZER_PIN = 3;
const int SDA_PIN = 4;
const int SCL_PIN = 5;
const int TX_PIN = 6;
const int RX_PIN = 7;
//It is not allowed to define pins 8-10 due to library conflicts, but I will still show them here for clarity
/*
const int SCK_PIN = 8;
const int MISO_PIN = 9;
const int MOSI_PIN = 10;
*/

//Buzzer drive functions
inline void dot()
{
  #if ENABLE_BUZZER
    digitalWrite(BUZZER_PIN,HIGH);
  #endif
  digitalWrite(LED_RED,LOW);
  delay(100);
  #if ENABLE_BUZZER
    digitalWrite(BUZZER_PIN,LOW);
  #endif
  digitalWrite(LED_RED,HIGH);
  delay(100);
}
inline void dash()
{
  #if ENABLE_BUZZER
    digitalWrite(BUZZER_PIN,HIGH);
  #endif
  digitalWrite(LED_RED,LOW);
  delay(500);
  #if ENABLE_BUZZER
    digitalWrite(BUZZER_PIN,LOW);
  #endif
  digitalWrite(LED_RED,HIGH);
  delay(100);
}

const char *morseCodeTable[10] = 
{ 
  "-----", ".----", "..---", "...--", "....-",  // 0-4
  ".....", "-....", "--...", "---..", "----."   // 5-9
};

/*
Device codes
This chart assigns each device a code, and if anything goes wrong MCU will broadcast the code in Morse.
  Device    |   Code
  MPU6050   |     0
  LSM6DS3   |     1
  TFCARD    |     2
  BMP085    |     3
  HMC5883L  |     4
  GNSS      |     5
Note that it is assumed the actuators and the buzzer are working by default
as it would be trivial to determine if they are working.
*/
//Error report function
//Reports device init error using buzzer, based on the error code above
//int -> void
void errorReport (int errorCode)
{
  for (const char *ptr = morseCodeTable[errorCode]; *ptr; ptr++)
  {
    (*ptr == '.') ? dot() : dash();
  }
}

//I2C
//MPU6050 IMU, using Adafruit library
//Note that the header file includes wire.h so wont be necessary to include it again
#include <Adafruit_MPU6050.h>
Adafruit_MPU6050 MPU6050;
Adafruit_Sensor *MPU6050_temp, *MPU6050_accel, *MPU6050_gyro;

float MPU6050_IMU_Data[6] = {0};                      //1D Array that stores 3-axis accel (m/s^2) and 3-axis gryo (rad/s) data
float MPU6050_Temperature = 0.0;                      //Temperature variable in degrees celcius
float MPU6050_Z_Bias = 0.0;                           //MPU6050 calibration factor for z-axis
//Function that retrieves sensor data from MPU6050 and updates them in global variables
void getMPU6050Data()
{
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  MPU6050_temp->getEvent(&temp);
  MPU6050_accel->getEvent(&accel);
  MPU6050_gyro->getEvent(&gyro);

  MPU6050_Temperature = temp.temperature;

  MPU6050_IMU_Data[0] = accel.acceleration.x;
  MPU6050_IMU_Data[1] = accel.acceleration.y;
  MPU6050_IMU_Data[2] = accel.acceleration.z - MPU6050_Z_Bias; //Z-axis Sensor offset

  MPU6050_IMU_Data[3] = gyro.gyro.x;
  MPU6050_IMU_Data[4] = gyro.gyro.y;
  MPU6050_IMU_Data[5] = gyro.gyro.z;
}

//LSM6DS3 IMU
#include "LSM6DS3.h"
LSM6DS3 IMU_BUILTIN(I2C_MODE, 0x6A);

float LSM6DS3_IMU_Data[6] = {0};                      //1D Array that stores 3-axis accel (m/s^2) and 3-axis gryo (rad/s) data
float LSM6DS3_Temperature = 0.0;                      //Temperature variable in degrees celcius
float LSM6DS3_Z_Bias = 0.0;                           //LSM6DS3 calibration constant for z-axis

//Function that retrieves sensor data from LSM6DS3 and updates them in global variables
void getLSM6DS3Data()
{
  LSM6DS3_Temperature = IMU_BUILTIN.readTempC();

  LSM6DS3_IMU_Data[0] = IMU_BUILTIN.readFloatAccelX();
  LSM6DS3_IMU_Data[1] = IMU_BUILTIN.readFloatAccelY();
  LSM6DS3_IMU_Data[2] = IMU_BUILTIN.readFloatAccelZ() - LSM6DS3_Z_Bias;

  LSM6DS3_IMU_Data[3] = IMU_BUILTIN.readFloatGyroX();
  LSM6DS3_IMU_Data[4] = IMU_BUILTIN.readFloatGyroY();
  LSM6DS3_IMU_Data[5] = IMU_BUILTIN.readFloatGyroZ();
}

//BMP085
#include <Adafruit_BMP085.h>
Adafruit_BMP085 bmp;

float BMP085_Temperature = 0.0;                     //Temperature data from BMP085 in degrees celcius
float BMP085_Pressure = 0.0;                        //Pressure data from BMP085 in pascals
float BMP085_Altitude = 0.0;                        //Calculated height data from BMP085 in meters

//Function that retrieves sensor data from BMP085 and updates them in global variables
void getBMP085Data()
{
  BMP085_Temperature = bmp.readTemperature();
  BMP085_Pressure = bmp.readPressure();
  BMP085_Altitude = bmp.readAltitude(LOCAL_SEA_LEVEL_PRESSURE);
}

float IMUWeight[2] = {0.2 , 0.8};             //IMU Weight array {MPU6050,LSM6DS3}
float TemperatureWeight[3] = {0, 0.7, 0.3};   //Temperature Weight array {MPU6050,LSM6DS3,BMP085}
float Processed_IMU_Data[6] = {0};            //Final output IMU data array {xaccel, yaccel, zaccel, xgyro, ygyro, zgyro}
float Processed_Temperature = 0.0;            //Final output Temperature in Celcius
float Processed_Pressure = 0.0;               //Final output Pressure in Pa
float Ground_Altitude = 0.0;                  //Ground altitude, will not be updated after airborne

// Kalman filter variables
float Estimated_Altitude = 0.0;                 // Estimated altitude
float Estimated_Vertical_Velocity = 0.0;        // Estimated vertical velocity
float estimationError[2][2] = {{1, 0}, {0, 1}}; // Error covariance
float measurementError = 2.0;                   // Barometer measurement noise
float processNoise = 0.05;                      // Process noise
float kalmanGain[2];                            // Kalman Gain
const float deltaTime = 0.1;                    // Time step in seconds

// Kalman Filter Function
void kalmanFilterAscentVelocity(float measuredAltitude, float accelZ) 
{
    // 1. Predict Step
    float Predicted_Altitude = Estimated_Altitude + Estimated_Vertical_Velocity * deltaTime + 0.5 * accelZ * deltaTime * deltaTime;
    float Predicted_Vertical_Velocity = Estimated_Vertical_Velocity + accelZ * deltaTime;
    
    estimationError[0][0] += processNoise;
    estimationError[1][1] += processNoise;
    estimationError[0][1] += processNoise * deltaTime;
    estimationError[1][0] += processNoise * deltaTime;

    // 2. Compute Kalman Gain
    float S = estimationError[0][0] + measurementError;           // Residual covariance
    kalmanGain[0] = estimationError[0][0] / S;
    kalmanGain[1] = estimationError[1][0] / S;

    // 3. Update Step
    float y = measuredAltitude - Predicted_Altitude;              // Measurement residual
    Estimated_Altitude = Predicted_Altitude + kalmanGain[0] * y;
    Estimated_Vertical_Velocity = Predicted_Vertical_Velocity + kalmanGain[1] * y;

    // 4. Update Error Covariance
    estimationError[0][0] *= (1 - kalmanGain[0]);
    estimationError[1][0] *= (1 - kalmanGain[1]);
    estimationError[0][1] = estimationError[1][0];
    estimationError[1][1] *= (1 - kalmanGain[1]);
}

//SPI
#include <SPI.h>

//TF card Memory module
#include <SD.h>
const char* FILE_NAME = "HAB_Data.txt";         //8-3 restriction

//Time related definitions
//Prepartion runtime variable, tracks time elapsed between startup and airborne in ms
//This should not be updated any longer after airborne
int preprationRunTime = 0;
//Effective runtime variable, tracks time elapsed after airborne in ms
int effectiveRunTime = 0;
//Timer function
void getTime(int status)
{
  //If not airborne, update Prepartion runtime only
  if (status == 0)
  {
    preprationRunTime = millis(); 
  }
  //If launched, update the flight time based on Prepartion runtime
  effectiveRunTime = millis() - preprationRunTime;
}

//Data record function, this should only be called once all post-processing of data are done
bool recordData(const char* fileName)
{
  char sensorData[512];
  char* ptr = sensorData;

  memset(sensorData, 0, sizeof(sensorData));

  ptr += sprintf(ptr, "Time:%lu\t", effectiveRunTime);               //Time data in seconds

  ptr += sprintf(ptr, "TEMP:%f\t", Processed_Temperature);                //Temperature data in degrees Celcius
  ptr += sprintf(ptr, "PRES:%d\t", Processed_Pressure);                   //Pressure data in Pascals

  ptr += sprintf(ptr, "ASCV:%f\t", Estimated_Vertical_Velocity);          //Estimated Ascent velocity in m/s
  ptr += sprintf(ptr, "ALTF:%f\t", Estimated_Altitude - Ground_Altitude); //Flight altitude in meters
  ptr += sprintf(ptr, "ALTS:%f\t", Estimated_Altitude);                   //Sea level altitude

  //Accelerometer data
  ptr += sprintf(ptr, "XACC:%f\t", Processed_IMU_Data[0]);
  ptr += sprintf(ptr, "YACC:%f\t", Processed_IMU_Data[1]);
  ptr += sprintf(ptr, "ZACC:%f\t", Processed_IMU_Data[2]);
  
  //Gyro data
  ptr += sprintf(ptr, "YAW:%f\t", Processed_IMU_Data[4]);
  ptr += sprintf(ptr, "PTCH:%f\t", Processed_IMU_Data[3]);
  ptr += sprintf(ptr, "ROLL:%f\t", Processed_IMU_Data[5]);

  // Check for buffer overflow
  if (ptr - sensorData >= sizeof(sensorData)) 
  {
    return false;
  }

  // Open file in append mode
  File dataFile = SD.open(fileName, O_WRITE | O_CREAT | O_APPEND);
  if (!dataFile) 
  {
    return false;
  }

  // Check if writing is successful
  if (!dataFile.println(sensorData)) 
  {
    return false;
  }

  dataFile.close();  // Always close file to prevent corruption
  return true;
}

//Flight progress variable
/*
0 = Standby on ground
1 = Ascent
2 = Descent
3 = Landed
*/
enum flightStatus 
{
  STNDBY, ASCENT, DESCNT, LANDED
};
static enum flightStatus currentStatus = STNDBY;
bool hasTerminatedFlight = false;
static int stationaryCount = 0;

void setup() 
{

  #if TEST_MODE
  Serial.begin(115200);
  #endif

  //Successful boot flag
  bool isSuccessfulBoot = true;

  //Actuator pins
  pinMode(ACTUATOR_PIN_1, OUTPUT);
  pinMode(ACTUATOR_PIN_2, OUTPUT);
  digitalWrite(ACTUATOR_PIN_1, LOW);
  digitalWrite(ACTUATOR_PIN_2, LOW);

  #if TEST_MODE
  digitalWrite(ACTUATOR_PIN_1, HIGH);
  digitalWrite(ACTUATOR_PIN_2, HIGH);
  delay(5000);
  digitalWrite(ACTUATOR_PIN_1, LOW);
  digitalWrite(ACTUATOR_PIN_2, LOW);
  #endif

  //Buzzer pin
  #if ENABLE_BUZZER
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
  #endif
  //LED indicator
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED,HIGH);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BLUE,HIGH);
  
  //I2C devices init
  //MPU6050
  if (!MPU6050.begin()) 
  {
    errorReport(0);
    isSuccessfulBoot = false;
    //If MPU6050 malfunctioned the sensor reading will be based on LSM6DS3 only
    IMUWeight[0] = 0;
    IMUWeight[1] = 1;
  }
  else
  {
    MPU6050_temp = MPU6050.getTemperatureSensor();
    MPU6050_temp->printSensorDetails();

    MPU6050_accel = MPU6050.getAccelerometerSensor();
    MPU6050_accel->printSensorDetails();

    MPU6050_gyro = MPU6050.getGyroSensor();
    MPU6050_gyro->printSensorDetails();
    
    //Dynamic 6050 calibration
    float MPU6050_sum = 0;
    const int numSamples = 100;
    for (int i = 0; i < numSamples; i++) 
    {
      getMPU6050Data();
      MPU6050_sum += MPU6050_IMU_Data[2]; // Read Z-axis acceleration
      delay(10); // Small delay to ensure stable readings
    }
    MPU6050_Z_Bias = MPU6050_sum / numSamples;
  }

  //LSM6DS3
  //Note this startup returns 0 on success
  if(IMU_BUILTIN.begin())
  {
    errorReport(1);
    isSuccessfulBoot = false;
    //If LSM6DS3 malfunctioned the sensor reading will be based on MPU6050 only
    IMUWeight[0] = 1;
    IMUWeight[1] = 0;
  }
  else
  {
    float LSM6DS3_sum = 0;
    const int numSamples = 100;
    for (int i = 0; i < numSamples; i++) 
    {
      getLSM6DS3Data();
      LSM6DS3_sum += LSM6DS3_IMU_Data[2];
      delay(10); // Small delay to ensure stable readings
    }
    LSM6DS3_Z_Bias = LSM6DS3_sum / numSamples;
  }

  //BMP085
  if(!bmp.begin())
  {
    errorReport(3);
    isSuccessfulBoot = false;
  }

  //SPI devices init
  if(!SD.begin(TF_CHIP_SELECT_PIN)) 
  {
    errorReport(2);
    isSuccessfulBoot = false;
  }

  //UART devices init
  //Function outputs 1 -> Success
  
  //If all devices are working there should be a blue light on
  //Else the red light would be on
  
  if (!isSuccessfulBoot)
  {
    digitalWrite(LED_RED,LOW);
  }
  else
  {
    digitalWrite(LED_BLUE,LOW);
    //OK
    dash();
    dash();
    dash();
    delay(500);

    dash();
    dot();
    dash();
  }
}


void loop() 
{
  //Get current time based on statusBMP085_Pressure
  getTime(currentStatus);

  //Retrieve sensor data
  getMPU6050Data();
  getLSM6DS3Data();
  getBMP085Data();

  //Evaluate final temperature, altitude and acceleration based on predefined weight
  for (int i = 0; i<6; i++)
  {
    Processed_IMU_Data[i] = IMUWeight[0]*MPU6050_IMU_Data[i] + IMUWeight[1]*LSM6DS3_IMU_Data[i];
  }
  Processed_Temperature = TemperatureWeight[0]*MPU6050_Temperature + TemperatureWeight[1]*LSM6DS3_Temperature + TemperatureWeight[2]*BMP085_Temperature;
  Processed_Pressure = BMP085_Pressure;

  kalmanFilterAscentVelocity(BMP085_Altitude, Processed_IMU_Data[2]);

  #if TEST_MODE
  Serial.print("ASCV:\t");
  Serial.print(Estimated_Vertical_Velocity);
  Serial.print("\t");

  Serial.print("ALTF:\t");
  Serial.print(Estimated_Altitude - Ground_Altitude);
  Serial.print("\t");

  Serial.println();
  #endif


  switch (currentStatus)
  {
    case STNDBY:
    {
      Ground_Altitude = Estimated_Altitude;                   //Update ground altitude
      //If in standby, detect takeoff conditions
      if (preprationRunTime > 20000)                          //Wait until sensor data stabilizes
      {
        if (Estimated_Vertical_Velocity > TAKEOFF_THRESHOLD)
        {
          currentStatus = ASCENT;
          #if TEST_MODE
          Serial.print("TAKEOFF DETECTED");
          Serial.println();
          #endif
          dash();
          break;
        }
      }
      break;
    }
    case ASCENT:
    {
      //Record data from this point onwards
      recordData(FILE_NAME);
      //Check flight termination conditions
      //Flight termination can only happen when the balloon is in ascent
      //and either runtime passed the limit or the altitude reached the limit
      bool flightTerminationCondition = (effectiveRunTime > MAX_FLIGHT_TIME) || (Estimated_Altitude - Ground_Altitude > MAX_FLIGHT_HEIGHT);
      if (flightTerminationCondition && !hasTerminatedFlight)
      {
        digitalWrite(ACTUATOR_PIN_2,HIGH);
        hasTerminatedFlight = !hasTerminatedFlight;
        #if TEST_MODE
        dash();
        Serial.print("FLT TERMINATION");
        Serial.println();
        #endif
      }
      if (Estimated_Vertical_Velocity < 0.01)
      {
        currentStatus = DESCNT;
        #if TEST_MODE
        dash();
        Serial.print("DESCENT DETECTED");
        Serial.println();
        #endif
      }
      break;
    }
    case DESCNT:
    {
      recordData(FILE_NAME);
      //Check if HAB has landed
      //If rate of change is sufficiently small for a period of time, a landing occurs
      {
        if (abs(Estimated_Vertical_Velocity) < 0.1) 
        {
            stationaryCount++;
        } 
        else 
        {
            stationaryCount = 0;
        }
        if (stationaryCount > LANDING_THRESHOLD) 
        {
          currentStatus = LANDED;
          #if TEST_MODE
          Serial.print("LANDING DETECTED");
          Serial.println();
          #endif
        }
      }
      break;
    }
    case LANDED:
    {
      //If landed, send Message ping every 1 min
      //L
      dot();
      dash();
      dot();
      dot();
      delay(500);

      //A
      dot();
      dash();
      delay(500);

      //N
      dash();
      dot();
      delay(500);

      //D
      dash();
      dot();
      dot();

      delay(60000);
      break;
    }
      
    default:
    {
      break;
    }
      
  }
}
