/* 2020 Patrick Corley - patrick.urban90@gmail.com
   CaptuRow Rowing speed monitor
   Migrated from Arduino Uno to Arduino MEGA 2560 on 7/9/2020

*/

#include <Wire.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <SPI.h>
#include "TinyGPS++.h"
#include "SparkFun_MMA8452Q.h"
#include "SdFat.h"
#include "rgb_lcd.h"
#include "TimerOne.h"
#include "arduinoFFT.h"

// Create Preprocessor defines...
#define GPS_RX_PIN 69
#define GPS_TX_PIN 68
#define CS 53
#define GPS_BAUD 9600
#define COM_PORT_BAUD 115200
#define DISTANCE_INTERVAL 5
#define STRK_ACCEL_THRES 0.5
#define START_CAPTURE_PIN 7
#define ELAPSED_TIME 1000
#define LOG_FILE_NAME "DATALOG.txt"

#define ACCEL_SAMPLE_NUM 64
#define ACCEL_SAMPLE_FREQ 1.33
#define ACCEL_SAMPLE_RATE_uS 1000000 / ACCEL_SAMPLE_FREQ

#define LCD_UPDATE_PERIOD_mS 1000

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

// Create program objects...
rgb_lcd lcd;
TinyGPSPlus gps;
MMA8452Q accel;
SdFat sd;
SdFile sd_file;
SoftwareSerial ss(GPS_TX_PIN, GPS_RX_PIN);
arduinoFFT fft = arduinoFFT(); 

// Create program variables...
double elap_mins = 0.00;
double elap_secs = 0.00;

double split_mins = 0.00;
double split_secs = 0.00;

double distance = 0.00;
double temp_distance = 0;
double distance_delta = 0;

int stroke_rate = 0;
long strk_counter = 0;

long log_line_cnt = 0;

double initial_lat = 0;
double initial_long = 0;

long last_lcd_update = 0;

double aX_samples[ACCEL_SAMPLE_NUM];
double aY_samples[ACCEL_SAMPLE_NUM];
double aZ_samples[ACCEL_SAMPLE_NUM];
long sample_idx = 0;

char current_lat[13] = "";
char current_long[13] = "";
char current_gps_speed[10] = "";
char aX[10]= "";
char aY[10] = "";
char aZ[10] = "";

bool DEBUG_EN = false;
bool CAP_STARTED = false;
bool FIRST_CAP = true;
bool CALC_FFT_FLAG = false;

char SD_BUF[256];

String log_file_name = "DATALOG.txt";

// Define functions
void setup() {
  // put your setup code here, to run once:
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.setRGB(0, 255, 0);
  ss.begin(GPS_BAUD);
  Serial.begin(COM_PORT_BAUD);
  accel.begin();
  accel.init(SCALE_2G, ODR_800);
  Timer1.initialize(ACCEL_SAMPLE_RATE_uS);
  Serial.println("Initializing SD card...");
  // see if the card is present and can be initialized:
  while (!sd.begin(CS, SPI_HALF_SPEED)){
    Serial.println("Waiting for SD card to be inserted...");
    delay(500);
  }
  Serial.println("card initialized.");
  delay(2000);
  // Init Variables
  strk_counter = 0;
  // Get initial location
  while(ss.available()) {
    gps.encode(ss.read());
    // Check if speed has been updated
    if (gps.location.isUpdated()) {
      initial_lat = gps.location.lat();
      initial_long = gps.location.lng();
    }
  }
}

void loop() {
  elap_mins = 0.00;
  elap_secs = 0.00;
  // Read software serial port and encode gps data
  process_distance();
  convert_speed();
  if (DEBUG_EN) {
    Serial.print("Speed Mins: ");
    Serial.print(String(split_mins));
    Serial.print(" Mins");
    Serial.print(" + ");
    Serial.print(String(split_secs));
    Serial.println(" Secs");
  }
  convert_time(&elap_mins, &elap_secs);
  // Update LCD
  if ((millis() - last_lcd_update) >= LCD_UPDATE_PERIOD_mS){
    last_lcd_update = millis();
    update_disp(distance, &elap_mins, &elap_secs);
  }
  // Update GPS 
  while(ss.available()) {
    gps.encode(ss.read());
    // Check if speed has been updated
    if (gps.speed.isUpdated()) {
      dtostrf(gps.speed.mps(), 8, 4, current_gps_speed);
    }
  }
  // Check status of SD capture switch
  if ((digitalRead(START_CAPTURE_PIN) == HIGH) and (CAP_STARTED == false)) {
      // Erase Datalog File only once when switch initally toggled
      while (!sd_file.open(LOG_FILE_NAME, O_WRONLY | O_CREAT | O_TRUNC)) {
        Serial.println("Waiting for SD card to be inserted...");
        delay(500);
      }
      Serial.println("Old Log data cleared");
      sd_file.close();
      // Opening for logging new data
      while (!sd_file.open(LOG_FILE_NAME, O_WRONLY | O_APPEND)) {
        Serial.println("Waiting for SD card to be inserted...");
        delay(500);
      }
      Timer1.attachInterrupt(write_to_sd);
      Serial.println("Logging new data...");
      CAP_STARTED = true;
  } else if ((digitalRead(START_CAPTURE_PIN) == LOW) and (CAP_STARTED == true)) {
      Timer1.detachInterrupt();
      sd_file.close();
      Serial.println("SD Closed!");
      CAP_STARTED = false;
      FIRST_CAP = true;
  }
  // Read accel
  if (accel.available()) {
      dtostrf(accel.getCalculatedX(), 9, 6, aX);
      dtostrf(accel.getCalculatedY(), 9, 6, aY);
      dtostrf(accel.getCalculatedZ(), 9, 6, aZ);
  }
  if (CALC_FFT_FLAG){
    calc_major_peak();
  }
}

void write_to_sd() {
  if ((digitalRead(START_CAPTURE_PIN) == HIGH) and (FIRST_CAP == false)){
    // Add in SD buffer and organize variables so everyhting can be written in one SD Write..
    aX_samples[sample_idx] =  atof(aX);
    aY_samples[sample_idx] =  atof(aY);
    aZ_samples[sample_idx] =  atof(aZ);
    if (sample_idx == ACCEL_SAMPLE_NUM - 1){
      sample_idx = 0;
      CALC_FFT_FLAG = true;
    }else{
      sample_idx++;
    }
    sprintf(SD_BUF, "Time(mS): %lu\tLog #: %lu\tLatitude: %s\tLongitude: %s\tX-Acceleration: %s\tY-Acceleration: %s\tZ-Acceleration: %s\tGPS Speed (KM/H): %s\n", millis(), log_line_cnt, current_lat, current_long, aX, aY, aZ, current_gps_speed);
    sd_file.print(SD_BUF);
    Serial.print(SD_BUF);
    log_line_cnt++;
  }else{
    FIRST_CAP = false;
  }
}

void calc_major_peak(void){
  // Function to calculate FFT of accelerometer samples
  double peak;
  double imag_arr[ACCEL_SAMPLE_NUM];
  // Build up imaginary data (zeros)
  for (int i = 0; i < ACCEL_SAMPLE_NUM; i++){
    imag_arr[i] = 0.0;
  }
  fft.Windowing(aX_samples, ACCEL_SAMPLE_NUM, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  fft.Compute(aX_samples, imag_arr, ACCEL_SAMPLE_NUM, FFT_FORWARD);
  fft.ComplexToMagnitude(aX_samples, imag_arr, ACCEL_SAMPLE_NUM);
  peak = fft.MajorPeak(aX_samples, ACCEL_SAMPLE_NUM, ACCEL_SAMPLE_FREQ);
  stroke_rate = (peak * 60) + 0.5;
  CALC_FFT_FLAG = false;
}

void process_distance() {
  double local_lat = gps.location.lat();
  double local_long = gps.location.lng();
  temp_distance = 0;
  distance_delta = 0;
  dtostrf(local_lat, 10, 6, current_lat);
  dtostrf(local_long, 11, 6, current_long);
  temp_distance = gps.distanceBetween(initial_lat, initial_long, local_lat, local_long);
  distance_delta = temp_distance - distance;
  if ((1000 > distance_delta) and (distance_delta > DISTANCE_INTERVAL)) {
    distance += distance_delta;
  }
  if (DEBUG_EN) {
    Serial.println("Initial Latitude = ");
    Serial.println(initial_lat, 6);
    Serial.println("Initial Longitude = ");
    Serial.println(initial_long, 6);
    Serial.println("Current Latitude = ");
    Serial.println(current_lat);
    Serial.println("Current Longitude = ");
    Serial.println(current_long);
    Serial.println("Current Distance from start point = ");
    Serial.println(temp_distance);
    Serial.println("Greatest distance from start point = ");
    Serial.println(distance);
  }
}

void convert_time(double *mins, double *secs) {
  double secs_factor;
  secs_factor = modf((((double) ELAPSED_TIME / 1000) / 60), mins);
  *secs = secs_factor * 60.00;

  if (DEBUG_EN) {
    Serial.println("mins = ");
    Serial.println(*mins);
    Serial.println("secs = ");
    Serial.println(*secs);
  }
}
void convert_speed() {
  double read_speed = 0;
  double result_factor = 0;
  double converted_speed = 0;
  double seconds_factor;

  read_speed = atof(current_gps_speed);
  result_factor = 500 / read_speed ;
  converted_speed = result_factor / 60.0000;
  seconds_factor = modf (converted_speed, &split_mins);
  split_secs = seconds_factor * 60.0000;

}
void update_disp(double distance, double* mins, double* secs) {
  // Print a message to the first line of LCD.
  lcd.setCursor(0, 0);
  //lcd.print("Split - ");
  //lcd.setCursor(8, 0);
  if (split_mins > 8) {
    lcd.print("WAITING...");
  } else {
    lcd.print(int(split_mins));
    lcd.setCursor(3, 0);
    lcd.print(":");
    lcd.setCursor(4, 0);
    lcd.print(split_secs);
  }
  // Print a message to the second line of LCD.
  lcd.setCursor(0, 1);
  lcd.print("D:");
  lcd.setCursor(2, 1);
  lcd.print(int(distance));
  lcd.setCursor(6, 1);

  if (true) {
    lcd.print("SR:");
    lcd.setCursor(11, 1);
    lcd.print(stroke_rate);
    //stroke_flag = false;
  } else {
    lcd.print((long) *mins);
    lcd.setCursor(8, 1);
    lcd.print("m");
    lcd.setCursor(10, 1);
    lcd.print(*secs);
    lcd.setCursor(15, 1);
    lcd.print("s");
  }
  //Serial.println(stroke_flag);
  //stroke_flag = !stroke_flag;
}

static void gps_smartdelay(unsigned long ms) {
  unsigned long start = millis();
  do
  {
    while (ss.available()) {
      // Always run this code regardless of delays
      gps.encode(ss.read());
      // Check if speed has been updated
      if (gps.speed.isUpdated()) {
        dtostrf(gps.speed.mps(), 6, 2, current_gps_speed);
        if (DEBUG_EN) {
          Serial.print("Current Speed = ");
          //Serial.print(current_gps_speed);
          Serial.print('0.00');
          Serial.println();
        }
      }
    }
  } while (millis() - start < ms);
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
  break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / ACCEL_SAMPLE_FREQ);
  break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * ACCEL_SAMPLE_FREQ) / ACCEL_SAMPLE_NUM);
  break;
    }
    Serial.print(abscissa, 6);
    if(scaleType==SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}
