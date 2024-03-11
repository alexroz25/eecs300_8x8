/*
  Read an 8x8 array of distances from the VL53L5CX
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 26, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to read all 64 distance readings at once.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/18642

*/

#include <Wire.h>

#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX

#define THRESHOLD_DETECT 0.8   // dist < 0.8 * calibrate => object_detected = true;
#define THRESHOLD_UNDETECT 0.9 // dist > 0.9 * calibrate => object_detected = false;

SparkFun_VL53L5CX myImager;
// Result data class structure, 1356 byes of RAM
VL53L5CX_ResultsData measurementData;

bool calibrated = false;
int16_t distance_calibrated[64];

int16_t distance_mm_prev[64];
int16_t distance_diff[64];
bool    object_detected[64];

int HEIGHT_MIN = -1219;
int HEIGHT_MAX = -1829;
bool anomaly_present = false;
int anomaly_index = 0;

int anomaly_size;
int16_t anomaly_xloc[64];
int16_t anomaly_yloc[64];

int person_count = 0;

int imageResolution = 0; //Used to pretty print output
int imageWidth = 0; //Used to pretty print output

void calibrate() { // init baseline (detecting nothing but the floor)
  for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth) {
    for (int x = imageWidth - 1 ; x >= 0 ; x--) {
      distance_calibrated[x + y] = measurementData.distance_mm[x + y];
    }
  }
}

void update_diff_matrix() {
  for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth) {
    for (int x = imageWidth - 1 ; x >= 0 ; x--) {
      // current - old
      distance_diff[x + y] = measurementData.distance_mm[x + y] - distance_mm_prev[x + y];
      // if significantly negative, then something has moved into this spot
      // if significantly positive, then something has moved out of this spot
      //Serial.print(distance_diff[x+y]);
      //Serial.print("\t");
    }
    Serial.println();
  }
}

void detect() {

  //Data to record for use later
  int size = 0;
  int firstXLoc = 0;
  int firstYLoc = 0;
  
  boolean currentAnomaly = false;
  //Determine if there is an anomaly
  for (int i = 0 ; i < 64; i++){
      if(distance_diff[i] < HEIGHT_MIN && distance_diff[i] > HEIGHT_MAX){
        object_detected[i] = true;
        //Record first seen locations
        Serial.print(object_detected[i]);
        Serial.print(" ");
        if(size == 0){
          firstXLoc = i/8;          
          firstYLoc = i-i/8;
        }
        Serial.print("(Size: ");
        Serial.print(size);
        size = size + 1;
        Serial.print(" ");
        Serial.print(size);
        Serial.print(") ");
        currentAnomaly = true;

      } else {
        object_detected[i] = false;
      }
  }
  Serial.print("Anomaly? - ");
  Serial.println(currentAnomaly);
  //Log Anomaly Data - point and size
  if(currentAnomaly){
      anomaly_index++;
      //Check for overflow
      if(anomaly_index > 63){
        Serial.println("ERROR! Anomaly Time overflow! Resetting!");
        anomaly_index = 0;
      }

    //save Data
    anomaly_size = anomaly_size + size;
    anomaly_xloc[anomaly_index] = firstXLoc;
    anomaly_yloc[anomaly_index] = firstYLoc;    
    
    Serial.print(anomaly_index);
    Serial.print(" ");
    Serial.print(anomaly_xloc[anomaly_index]);
    Serial.print(" ");
    Serial.println(anomaly_size);
  }  
  //If Previous anomaly is no longer there, Perform analysis
  if(!currentAnomaly && anomaly_present){     
     // Was it a person?

      double avg_size = anomaly_size/(anomaly_index+0.0);
      Serial.print("Size - ");
      Serial.println(avg_size);
    if(avg_size >= 2 && avg_size <= 15){ //&& avg_size <7){
      //Count can change once in here
      //entrance Conditions
      Serial.print("Deciding on Direction: ");
      Serial.print(anomaly_xloc[anomaly_index]);
      Serial.print(" > ");
      Serial.println(anomaly_xloc[1]);      
      if(anomaly_xloc[anomaly_index] > anomaly_xloc[1]){
        Serial.println("adding person");
        person_count++;
      } else if(anomaly_xloc[anomaly_index] < anomaly_xloc[1]){
        Serial.println("Subtracting person");
        person_count--;
        
      }
    } 

    //clear data
    anomaly_index = 0;
    anomaly_size = 0;
    for(int i = 0; i < 64; i++){
      anomaly_xloc[i] = 0;
      anomaly_yloc[i] = 0;
    }
    anomaly_present = false;
  }

  anomaly_present = currentAnomaly;
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("SparkFun VL53L5CX Imager Example");

  Wire.begin(); //This resets to 100kHz I2C
  Wire.setClock(400000); //Sensor has max I2C freq of 400kHz 
  
  Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");
  if (myImager.begin() == false)
  {
    Serial.println(F("Sensor not found - check your wiring. Freezing"));
    while (1) ;
  }
  
  myImager.setResolution(8*8); //Enable all 64 pads
  
  imageResolution = myImager.getResolution(); //Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution); //Calculate printing width

  myImager.startRanging();
}

void loop()
{
  //Poll sensor for new data
  if (myImager.isDataReady() == true) {

    if (!calibrated) {
      calibrate();
    }

   

    // gets current data into int16_t distance_mm_prev array
    if (myImager.getRangingData(&measurementData)) //Read distance data into array
    {
       update_diff_matrix();

        detect();

      Serial.print("Person Count:");
      Serial.println(person_count);

      Serial.println();

      //The ST library returns the data transposed from zone mapping shown in datasheet
      //Pretty-print data with increasing y, decreasing x to reflect reality
      for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth)
      {
        for (int x = imageWidth - 1 ; x >= 0 ; x--)
        {
          distance_mm_prev[x + y] = measurementData.distance_mm[x + y];
        }
        //Serial.println();
      }
      //Serial.println();
    }
  }

  delay(100); //Small delay between polling
}
