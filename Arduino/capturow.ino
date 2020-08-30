/* 2020 Patrick Corley - patrick.urban90@gmail.com
 * CaptuRow Rowing speed monitor 
 *
 */


#include <Wire.h>
#include "rgb_lcd.h"
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <math.h>  
#include "SparkFun_MMA8452Q.h"  
#include <SPI.h>
#include "SdFat.h"

#define GPS_RX_PIN 8
#define GPS_TX_PIN 9
#define CS 10
#define GPS_BAUD 9600
#define COM_PORT_BAUD 115200
#define DISTANCE_INTERVAL 5
#define STRK_ACCEL_THRES 0.5
#define START_CAPTURE_PIN 7
#define MIN_MILLIS_BETWEEN_STRKS 1333

#define LOG_FILE_NAME "DATALOG.txt"

rgb_lcd lcd;
TinyGPSPlus gps;
MMA8452Q accel; 
SdFat sd;
SdFile sd_file;
SoftwareSerial ss(GPS_TX_PIN, GPS_RX_PIN);

const int colorR = 0;
const int colorG = 255;
const int colorB = 0;

long elapsed_time = 0;
bool stroke_flag = true;

double split_mins = 0.00;
double split_secs = 0.00;
double current_gps_speed = 0.00;
double distance = 0.00;
long strk_counter = 0;
bool strk_meas = true;
long t_strk_measured = 0;

long log_line_cnt = 0;

double initial_lat, initial_long, current_lat, current_long = 0.00;

double aX;
double aY;
double aZ;

bool DEBUG_EN = false;
bool begin_piece = false;
bool capture_switch_toggle = false;

String log_file_name = "DATALOG.txt";

void setup() {
  // put your setup code here, to run once:
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.setRGB(colorR, colorG, colorB);
  ss.begin(GPS_BAUD);
  Serial.begin(COM_PORT_BAUD);
  accel.begin();
  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
 if (!sd.begin(CS, SPI_HALF_SPEED)) sd.initErrorHalt();
 Serial.println("card initialized.");
 if (!sd_file.open(LOG_FILE_NAME, O_WRONLY | O_CREAT | O_TRUNC)) {
    sd.errorHalt("opening test.txt for write failed");
 }
 sd_file.close();
 Serial.println("Old Log data cleard");

  // Init Variables
  aX = 999;
  aY = 999;
  aZ = 999;
  strk_counter = 0;
  gps_smartdelay(2000);
  // Get intial location
  initial_lat = gps.location.lat();
  initial_long = gps.location.lng();
  elapsed_time = 0;
}

void loop() {

  double elap_mins = 0.00;
  double elap_secs = 0.00;
  strk_meas = true;
  // Read software serial port and encode gps data
  process_distance();
  convert_speed();
  if (DEBUG_EN){
    Serial.print("Speed Mins: ");
    Serial.print(String(split_mins));
    Serial.print(" Mins");
    Serial.print(" + ");
    Serial.print(String(split_secs));
    Serial.println(" Secs");
  }
  elapsed_time = millis();
  if (DEBUG_EN){
    Serial.println("Elapsed_time = ");
    Serial.println(elapsed_time);
  }
  convert_time(&elap_mins, &elap_secs);
  update_disp(distance, strk_counter, &elap_mins, &elap_secs);
  gps_smartdelay(1);
  if (digitalRead(START_CAPTURE_PIN) == HIGH){
    if (capture_switch_toggle){
      capture_switch_toggle = false;
    }
    write_to_sd();
  }else{
    capture_switch_toggle = true;
    sd_file.close();
  }
}


void write_to_sd(){

  // if the file is available, write to it:
  // open the file for write at end like the Native SD library
  if (!sd_file.open(LOG_FILE_NAME, O_WRONLY | O_APPEND)) {
    sd.errorHalt("Opening SD file failed!");
  }
  
  sd_file.print("Time(mS): ") ;
  sd_file.print(millis());
  sd_file.print("\t"); 
  
  sd_file.print("Log #: ") ;
  sd_file.print(log_line_cnt);
  sd_file.print("\t"); 
  
  sd_file.print("Latitude: "); 
  sd_file.print(current_lat, 8);
  sd_file.print("\t");
  
  sd_file.print("Longitude: ");
  sd_file.print(current_long, 8);
  sd_file.print("\t");
  
  sd_file.print("Number of Strokes: ");
  sd_file.print(strk_counter);
  sd_file.print("\t");
  
  sd_file.print("X-Acceleration: ");
  sd_file.print(aX, 4);
  sd_file.print("\t");

  sd_file.print("Y-Acceleration: ");
  sd_file.print(aY, 4);
  sd_file.print("\t");

  sd_file.print("Z-Acceleration: ");
  sd_file.print(aZ, 4);
  sd_file.print("\t");

  sd_file.print("Distance Travelled: ");
  sd_file.print(distance, 4);
  sd_file.print("\t");
  
  sd_file.print("Split: ");
  sd_file.print(split_mins, 0);
  sd_file.print(":");
  sd_file.print(split_secs);
  sd_file.print("\t");
  
  sd_file.print("GPS Speed (KM/H): ");
  sd_file.print(current_gps_speed);
  sd_file.print("\n");
    
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
//    Serial.print("\t");
//    Serial.print("Distance Travelled: ");
//    Serial.print(distance, 4);
//    Serial.print("\t");
//    Serial.print("Split: ");
//    Serial.print(split_mins, 0);
//    Serial.print(":");
//    Serial.print(split_secs);
//    Serial.print("GPS Speed (KM/H): ");
//    Serial.print(current_gps_speed);
//    Serial.print("\n");
  sd_file.close();
    
  //   
  log_line_cnt++;
}
void process_accel(){
  // Accelerometer data rate = 800Hz by default -> 800 readings per second.
  // Therefore, if we analyze 200 samples for rapid change in acceleration 
  // On any axes. We will measure over ~250mS.
  double temp_aX, temp_aY, temp_aZ;
  double delta_aX, delta_aY, delta_aZ;
  if (accel.available()) {

    aX = accel.getCalculatedX();
    aY = accel.getCalculatedY();
    aZ = accel.getCalculatedZ();

    if (((abs(aX) >= STRK_ACCEL_THRES) || (abs(aY) >= STRK_ACCEL_THRES) || (abs(aZ) >= 3))){
      if ((millis() - t_strk_measured) >= MIN_MILLIS_BETWEEN_STRKS){
        t_strk_measured = millis();
        Serial.println();
        Serial.println("Stroke!");
        Serial.println();
        strk_counter++;
      }
    }
  } 
}

void process_distance(){
  double temp_distance = 0;
  double distance_delta = 0;
  current_lat = gps.location.lat();
  current_long = gps.location.lng();
  temp_distance = gps.distanceBetween(initial_lat, initial_long, current_lat, current_long);
  distance_delta = temp_distance - distance;
  if ((1000 > distance_delta) and (distance_delta > DISTANCE_INTERVAL)){
    distance += distance_delta;
  }
  if (DEBUG_EN){
    Serial.println("Initial Latitude = ");
    Serial.println(initial_lat, 6);
    Serial.println("Initial Longitude = ");
    Serial.println(initial_long, 6);
    Serial.println("Current Latitude = ");
    Serial.println(current_lat, 6);
    Serial.println("Current Longitude = ");
    Serial.println(current_long, 6);
    Serial.println("Current Distance from start point = ");
    Serial.println(temp_distance);
    Serial.println("Greatest distance from start point = ");
    Serial.println(distance);
  }
}

void convert_time(double *mins, double *secs){
  double secs_factor;
  secs_factor = modf((((double) elapsed_time / 1000) / 60), mins);
  *secs = secs_factor * 60.00;
  
  if (DEBUG_EN){
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
  
  read_speed = current_gps_speed;
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
  if (split_mins > 8){
    lcd.print("WAITING...");
  }else{
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
 
  if (stroke_flag == true){
    lcd.print("SR:");
    lcd.setCursor(11, 1);
    lcd.print(stroke_rate);
    //stroke_flag = false;
  }else if(stroke_flag == false){
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

static void gps_smartdelay(unsigned long ms){
  unsigned long start = millis();
  do 
  {
    while (ss.available()){
      // Always run this code regardless of delays
      gps.encode(ss.read());
      // Check if speed has been updated
      if (gps.speed.isUpdated()){
        current_gps_speed = gps.speed.mps();
        if (DEBUG_EN){
          Serial.print("Current Speed = ");
          Serial.print(current_gps_speed);
          Serial.println();
        }
      }
    }
    process_accel();
  } while (millis() - start < ms);
}
