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
#include <deque>

#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX


#define THRESHOLD_DETECT 0.8   // dist < 0.8 * calibrate => object_detected = true;
#define THRESHOLD_UNDETECT 0.9 // dist > 0.9 * calibrate => object_detected = false;
#define THRESHOLD_DIFF_MOVE 0.4 // diff_matrix > 0.2 * calibration = leaving square, diff_matrix < -0.2 * calibration = entering square
#define DEQUE_SIZE 6

typedef struct pair {
  //       x
  //     0 1 2
  //   0 d d d
  // y 1 d d d
  //   2 d d d
  int y;
  int x;
  pair() {
    y = -1;
    x = -1;
  }
  pair(int yi, int xi) {
    y = yi;
    x = xi;
  }
} pair_t;

typedef struct bounding_box {
  std::deque<pair_t> pos_history; // store last DEQUE_SIZE items
  bounding_box(){}
} bounding_box_t;

SparkFun_VL53L5CX myImager;
// Result data class structure, 1356 byes of RAM
VL53L5CX_ResultsData measurementData;

bounding_box_t objects[8];
int16_t objects_count = 0; // # people currently being detected

bool calibrated = false;//  y  x
int16_t distance_calibrated[8][8];

int16_t distance_mm_prev[8][8];
int16_t distance_diff[8][8];
bool    object_detected[8][8];

int16_t person_count = 0;
int16_t prev_person_count = 0;

int imageResolution = 0; //Used to pretty print output
int imageWidth = 0; //Used to pretty print output

void calibrate() { // init baseline (detecting nothing but the floor)
  for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth) {
    for (int x = 0; x < imageWidth; x++) {
      distance_calibrated[y/imageWidth][x] = measurementData.distance_mm[x + y];
    }
  }
  calibrated = true;
}

void update_diff_matrix() {
  for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth) {
    for (int x = 0; x < imageWidth; x++) {
      // current - old
      distance_diff[y/imageWidth][x] = measurementData.distance_mm[x + y] - distance_mm_prev[y/imageWidth][x];
      distance_diff[y/imageWidth][x] = distance_diff[y/imageWidth][x] < THRESHOLD_DIFF_MOVE * -1 * distance_calibrated[y/imageWidth][x] ? -1 : 
                              distance_diff[y/imageWidth][x] > THRESHOLD_DIFF_MOVE * distance_calibrated[y/imageWidth][x] ? 1 : 0;
      // if significantly negative, then something has moved into this spot
      // if significantly positive, then something has moved out of this spot
    }
  }
}

int calculate_dir_mag(bounding_box_t &b) {
  int result = 0;
  for (int i = 0; i < DEQUE_SIZE-1; i++) {
    int y_new = b.pos_history[i+1].y;
    int y_old = b.pos_history[i].y;
        // new data           // old data
    if (y_new != -1  && y_old != -1) {
      result = y_new > y_old ? result+1 : y_new < y_old ? result-1 : result;
    }
  }
  return result;
}

bool check_past_threshold(bounding_box_t &b, int t) {
  for (int i = 0; i < DEQUE_SIZE-1; i++) {
    if (b.pos_history[i].y == t) return true;
  }
  return false;
}

void detect() {
  for (int y = 0; y <= imageWidth * (imageWidth - 1); y+= imageWidth) {
    for (int x = 0; x < imageWidth; x++) {
      object_detected[y/imageWidth][x] = measurementData.distance_mm[x + y] < (distance_calibrated[y/imageWidth][x] * THRESHOLD_DETECT) ? true :
                                measurementData.distance_mm[x + y] > (distance_calibrated[y/imageWidth][x] * THRESHOLD_UNDETECT) ? false : object_detected[y/imageWidth][x];
    }
  }

  // scan for single blob
  int minY = 8;
  int minX = 8;
  int maxY = -1;
  int maxX = -1;
  for (int y = 0; y < 8; y++) {
    for (int x = 0; x < 8; x++) {
      if (object_detected[y][x]) {
        minY = y < minY ? y : minY;
        minX = x < minX ? x : minX;
        maxY = y > maxY ? y : maxY;
        maxX = x > maxX ? x : maxX;
      }
    }
  }
  if (minY != 8) {
    objects[0].pos_history.emplace_back((minY+maxY)/2,(minX+maxX)/2);
    if (objects[0].pos_history.size() > DEQUE_SIZE) objects[0].pos_history.pop_front();
  
    #define Y_LINE 2
    if (objects[0].pos_history.back().y == Y_LINE && calculate_dir_mag(objects[0]) >= 1 && !check_past_threshold(objects[0],Y_LINE)) {
      person_count++;
    }
    else if (objects[0].pos_history.back().y == Y_LINE && calculate_dir_mag(objects[0]) <= -1 && !check_past_threshold(objects[0],Y_LINE)) {
      person_count--;
    }
    #undef Y_LINE
  }
  
  
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
  
  bool response = myImager.setRangingFrequency(15);
  if (response == true)
  {
    int frequency = myImager.getRangingFrequency();
    if (frequency > 0)
    {
      Serial.print("Ranging frequency set to ");
      Serial.print(frequency);
      Serial.println(" Hz.");
    }
    else
      Serial.println(F("Error recovering ranging frequency."));
  }
  else
  {
    Serial.println(F("Cannot set ranging frequency requested. Freezing..."));
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

    // gets current data into int16_t distance_mm_prev array
    if (myImager.getRangingData(&measurementData)) //Read distance data into array
    {

      if (!calibrated) {
        calibrate();
        Serial.println("Calibrated");
      }

      update_diff_matrix();

      detect();

      //The ST library returns the data transposed from zone mapping shown in datasheet
      //Pretty-print data with increasing y, decreasing x to reflect reality
      for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth)
      {
        for (int x = imageWidth-1; x >= 0; x--)
        {
          distance_mm_prev[y/imageWidth][x] = measurementData.distance_mm[x + y];
          //Serial.print("\t");
          //Serial.print(distance_diff[y/imageWidth][x]);
          //Serial.print(object_detected[y/imageWidth][x]);
          //Serial.print(measurementData.distance_mm[x+y]);
        }
        //Serial.println();
      }
      //Serial.println();
      if (person_count != prev_person_count) {
        Serial.print("person_count: ");
        Serial.print(person_count);
        prev_person_count = person_count;
        Serial.println();
      }
    }
  }

  delay(5); //Small delay between polling

}
