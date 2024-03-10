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
    }
  }
}

void detect() {
  
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

    update_diff_matrix();

    detect();

    // gets current data into int16_t distance_mm_prev array
    if (myImager.getRangingData(&measurementData)) //Read distance data into array
    {
      //The ST library returns the data transposed from zone mapping shown in datasheet
      //Pretty-print data with increasing y, decreasing x to reflect reality
      for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth)
      {
        for (int x = imageWidth - 1 ; x >= 0 ; x--)
        {
          distance_mm_prev[x + y] = measurementData.distance_mm[x + y];
        }
        Serial.println();
      }
      Serial.println();
    }
  }

  delay(5); //Small delay between polling
}
