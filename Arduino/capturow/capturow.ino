/* 2020 Patrick Corley - patrick.urban90@gmail.com
   CaptuRow Rowing speed monitor

*/

#include <Wire.h>
#include "rgb_lcd.h"
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <math.h>
#include "SparkFun_MMA8452Q.h"
#include <SPI.h>
#include "SdFat.h"
#include "TimerOne.h"

#define GPS_RX_PIN 8
#define GPS_TX_PIN 9
#define CS 10
#define GPS_BAUD 9600
#define COM_PORT_BAUD 115200
#define DISTANCE_INTERVAL 5
#define STRK_ACCEL_THRES 0.5
#define START_CAPTURE_PIN 7
#define ACCEL_SAMPLE_RATE_uS 505000
#define ELAPSED_TIME 1000
#define LOG_FILE_NAME "DATALOG.txt"

rgb_lcd lcd;
TinyGPSPlus gps;
MMA8452Q accel;
SdFat sd;
SdFile sd_file;
SoftwareSerial ss(GPS_TX_PIN, GPS_RX_PIN);


double elap_mins = 0.00;
double elap_secs = 0.00;

double split_mins = 0.00;
double split_secs = 0.00;
char current_gps_speed[7];
double distance = 0.00;
double temp_distance = 0;
double distance_delta = 0;
long strk_counter = 0;

long log_line_cnt = 0;

char initial_lat[11];
char initial_long[12];
char current_lat[11];
char current_long[12];

char aX[8];
char aY[8];
char aZ[8];
//double aX = 0 ;
//double aY = 0;
//double aZ = 0;

bool DEBUG_EN = false;
bool CAP_STARTED = false;
bool FIRST_CAP = true;

char SD_BUF[70];

String log_file_name = "DATALOG.txt";

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

  // Init Variables
  strk_counter = 0;
  // Get intial location
  dtostrf(gps.location.lat(), 10, 6, initial_lat);
  dtostrf(gps.location.lng(), 11, 6, initial_long);

}

void loop() {
  elap_mins = 0.00;
  elap_secs = 0.00;
  // Read software serial port and encode gps data
  process_distance();
  convert_speed();
//  Serial.println("I have just updated the accelerometer data!");
//  Serial.print(millis());
//  Serial.print("\n");
    if (DEBUG_EN) {
    Serial.print("Speed Mins: ");
    Serial.print(String(split_mins));
    Serial.print(" Mins");
    Serial.print(" + ");
    Serial.print(String(split_secs));
    Serial.println(" Secs");
  }

  convert_time(&elap_mins, &elap_secs);
  update_disp(distance, strk_counter, &elap_mins, &elap_secs);
  // Update GPS 
  while(ss.available()) {
    gps.encode(ss.read());
    // Check if speed has been updated
    if (gps.speed.isUpdated()) {
      dtostrf(gps.speed.mps(), 6, 2, current_gps_speed);
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
      dtostrf(accel.getCalculatedX(), 7, 4, aX);
      dtostrf(accel.getCalculatedX(), 7, 4, aY);
      dtostrf(accel.getCalculatedX(), 7, 4, aZ);
  }
}

void write_to_sd() {
  
  if ((digitalRead(START_CAPTURE_PIN) == HIGH) and (FIRST_CAP == false)){
    // Add in SD buffer and organize variables so everyhting can be written in one SD Write..
    //sprintf(SD_BUF, "Time(mS): %d\tLog #: %d\tLatitude: %f\tLongitude: %f\tX-Acceleration: %f\tGPS Speed (KM/H): %f\n", log_time, log_line_cnt, current_lat, current_long, aX, current_gps_speed);
    sprintf(SD_BUF, "Time(mS): %lu\tX-Acceleration: %s\tY-Acceleration: %s\tZ-Acceleration: %s\n", millis(), aX, aY, aZ);
    sd_file.print(SD_BUF);
    //sd_file.print(SD_BUF);
//    sd_file.print("Time(mS): ") ;
//    sd_file.print(log_time);
//    sd_file.print("\t");
//  
//    sd_file.print("Log #: ") ;
//    sd_file.print(log_line_cnt);
//    sd_file.print("\t");
//  
//    sd_file.print("Latitude: ");
//    sd_file.print(current_lat, 8);
//    sd_file.print("\t");
//  
//    sd_file.print("Longitude: ");
//    sd_file.print(current_long, 8);
//    sd_file.print("\t");
//  
//    sd_file.print("Number of Strokes: ");
//    sd_file.print(strk_counter);
//    sd_file.print("\t");
//  
//    sd_file.print("X-Acceleration: ");
//    sd_file.print(aX, 4);
//    sd_file.print("\t");
//  
//    sd_file.print("Y-Acceleration: ");
//    sd_file.print(aY, 4);
//    sd_file.print("\t");
//  
//    sd_file.print("Z-Acceleration: ");
//    sd_file.print(aZ, 4);
//    sd_file.print("\t");
  
//    sd_file.print("Distance Travelled: ");
//    sd_file.print(distance, 4);
//    sd_file.print("\t");
//  
//    sd_file.print("Split: ");
//    sd_file.print(split_mins, 0);
//    sd_file.print(":");
//    sd_file.print(split_secs);
//    sd_file.print("\t");
  
//    sd_file.print("GPS Speed (KM/H): ");
//    sd_file.print(current_gps_speed);
//    sd_file.print("\n");
  
  //  Serial.print("Log #: ") ;
  //  Serial.print(log_line_cnt);
  //  Serial.print("\t");
  //  Serial.print("Latitude: ");
  //  Serial.print(current_lat, 8);
  //  Serial.print("\t");
  //  Serial.print("Longitude: ");
  //  Serial.print(current_long, 8);
  //  Serial.print("\t");
  //  Serial.print("Number of Strokes: ");
  //  Serial.print(strk_counter);
  //  Serial.print("\t");
  //  Serial.print("X-Acceleration: ");
  //  Serial.print(aX, 4);
  //  Serial.print("\t");
  //  Serial.print("Y-Acceleration: ");
  //  Serial.print(aY, 4);
  //  Serial.print("\t");
  //  Serial.print("Z-Acceleration: ");
  //  Serial.print(aZ, 4);
  //  Serial.print("\n");
  //  Serial.print("\t");
  //  Serial.print("Distance Travelled: ");
  //  Serial.print(distance, 4);
  //  Serial.print("\t");
  //  Serial.print("Split: ");
  //  Serial.print(split_mins, 0);
  //  Serial.print(":");
  //  Serial.print(split_secs);
  //  Serial.print("GPS Speed (KM/H): ");
  //  Serial.print(current_gps_speed);
  //  Serial.print("\n");
  //
    log_line_cnt++;
  }else{
    FIRST_CAP = false;
  }
}

void process_distance() {
  temp_distance = 0;
  distance_delta = 0;
  dtostrf(gps.location.lat(), 10, 6, current_lat);
  dtostrf(gps.location.lng(), 11, 6, current_long);
//  temp_distance = gps.distanceBetween(initial_lat, initial_long, current_lat, current_long);
  distance_delta = temp_distance - distance;
  if ((1000 > distance_delta) and (distance_delta > DISTANCE_INTERVAL)) {
    distance += distance_delta;
  }
  if (DEBUG_EN) {
//    Serial.println("Initial Latitude = ");
//    Serial.println(initial_lat, 6);
//    Serial.println("Initial Longitude = ");
//    Serial.println(initial_long, 6);
//    Serial.println("Current Latitude = ");
//    Serial.println(current_lat, 6);
//    Serial.println("Current Longitude = ");
//    Serial.println(current_long, 6);
//    Serial.println("Current Distance from start point = ");
//    Serial.println(temp_distance);
//    Serial.println("Greatest distance from start point = ");
//    Serial.println(distance);
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

  //read_speed = atof(current_gps_speed);
  read_speed = 1.1;
  result_factor = 500 / read_speed ;
  converted_speed = result_factor / 60.0000;
  seconds_factor = modf (converted_speed, &split_mins);
  split_secs = seconds_factor * 60.0000;

}
void update_disp(double distance, long stroke_rate, double* mins, double* secs) {
  lcd.clear();
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
          Serial.print(current_gps_speed);
          Serial.println();
        }
      }
    }
  } while (millis() - start < ms);
}
