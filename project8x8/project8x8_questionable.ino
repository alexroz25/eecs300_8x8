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
#include <vector>

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
    y = -2;
    x = -2;
  }
  pair(int yi, int xi) {
    y = yi;
    x = xi;
  }
} pair_t;

typedef struct peak {
  std::deque<pair_t> pos_history;
  bool active;
  peak() {
    active = false;
  }
  peak(pair_t init) {
    active = true;
    pos_history.push_back(init);
  }
} peak_t;

SparkFun_VL53L5CX myImager;
// Result data class structure, 1356 byes of RAM
VL53L5CX_ResultsData measurementData;

std::vector<peak_t> peaks;
int16_t objects_count = 0; // # people currently being detected

bool calibrated = false;//  y  x
int16_t distance_calibrated[8][8];

std::vector<std::vector<int16_t>> distance_curr(8,std::vector<int16_t>(8));
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

// Finding peak element in a 2D Array.
std::vector<pair_t> findPeakGrid()
{
    std::vector<pair_t> result;
    int row = distance_curr.size();
    int column = distance_curr[0].size();
 
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < column; j++) {
            // checking with top row
            if (i > 0) {
              if (distance_curr[i][j] >= distance_curr[i-1][j]) continue;
              if (j > 0 && distance_curr[i][j] >= distance_curr[i-1][j-1]) continue;
              if (j < column - 1 && distance_curr[i][j] >= distance_curr[i-1][j+1]) continue;
            }
            // checking with right element
            if (j < column - 1)
                if (distance_curr[i][j] >= distance_curr[i][j + 1])
                    continue;
            // checking with bottom row
            if (i < row - 1) {
              if (distance_curr[i][j] >= distance_curr[i + 1][j]) continue;
              if (j > 0 && distance_curr[i][j] >= distance_curr[i + 1][j-1]) continue;
              if (j < column - 1 &&  distance_curr[i][j] >= distance_curr[i + 1][j+1]) continue;
            }
            // checking with left element
            if (j > 0)
                if (distance_curr[i][j] >= distance_curr[i][j - 1])
                    continue;
 
            result.emplace_back(i,j);
        }
    }

    std::vector<pair_t> real;
    for (int i = 0; i < result.size(); i++) {
      if (object_detected[result[i].y][result[i].x]) real.push_back(result[i]);
    }
    return real;
} // https://www.geeksforgeeks.org/find-peak-element-2d-array/

bool within_one(pair_t a, pair_t b) {
  if ((a.x-b.x == -1 || a.x-b.x == 0 || a.x-b.x == 1) && (a.y-b.y == -1 || a.y-b.y == 0 || a.y-b.y == 1)) {
    return true;
  }
  return false;
}

// passes in a vector of peaks currently being detected
// should edit vector<pair_t> peaks such that if we find an element of v near a previously active peak
// we should add that v to the pos_history of peaks[i]
// otherwise, if we dont find any previous peaks near v, its a new peak
// additionally, if we have a peak which we find no current vectors near, we should deactivate it
void assign_peaks(std::vector<pair_t> v) {

  // loop through previously detected peaks
  for (int i = 0; i < peaks.size(); i++) {
    // check for current peaks which are within 1
    bool assigned = 0;
    for (int j = 0; j < v.size(); j++) {
      // new peak found to be next to a previous peak
      if (peaks[i].active && within_one(peaks[i].pos_history.back(),v[j]) && !assigned) {
        assigned = 1;
        peaks[i].pos_history.push_back(v[j]);
        if (peaks[i].pos_history.size() > 6) {
          peaks[i].pos_history.pop_front();
        }
      }
    }
    // deactivate peaks
    if (peaks[i].active && !assigned) {
      peaks.erase(peaks.begin()+i);
      i--;
    }
  }

  // activate peaks
  std::vector<bool> v_found;
  for (int i = 0; i < v.size(); i++) {
    bool found = 0;
    for (int j = 0; j < peaks.size(); j++) {
      if (peaks[j].active && within_one(peaks[j].pos_history.back(),v[i])) {
        found = 1;
      }
    }
    v_found.push_back(found);
  }
  // keep activating peaks
  for (int i = 0; i < v_found.size(); i++) {
    if (!v_found[i]) {
      peaks.emplace_back(v[i]);
    }
  }
} 

int calculate_dir_mag(std::deque<pair_t> d) {
  int result = 0;
  for (int i = 0; i < d.size()-1; i++) {
    int y_new = d[i+1].y;
    int y_old = d[i].x;
    if (y_new > y_old) result++;
    else if (y_new < y_old) result--;
  }
  return result;
}

bool check_prev_threshold(std::deque<pair_t> d, int t) {
  if (d.size() < 2) return true;
  for (int i = 0; i < d.size()-1; i++) {
    if (d[i].y == t) return true;
  }
  return false;
}

void count_people() {
  for (int i = 0; i < peaks.size(); i++) {
    if (peaks[i].active) {
      if (peaks[i].pos_history.back().y == 3 && calculate_dir_mag(peaks[i].pos_history) >= 1 && !check_prev_threshold(peaks[i].pos_history,3)) {
        person_count++;
      }
      else if (peaks[i].pos_history.back().y == 3 && calculate_dir_mag(peaks[i].pos_history) <= -1 && !check_prev_threshold(peaks[i].pos_history,3)) {
        person_count--;
      }
    }
  }
}

void detect() {
  for (int y = 0; y <= imageWidth * (imageWidth - 1); y+= imageWidth) {
    for (int x = 0; x < imageWidth; x++) {
      object_detected[y/imageWidth][x] = measurementData.distance_mm[x + y] < (distance_calibrated[y/imageWidth][x] * THRESHOLD_DETECT) ? true :
                                measurementData.distance_mm[x + y] > (distance_calibrated[y/imageWidth][x] * THRESHOLD_UNDETECT) ? false : object_detected[y/imageWidth][x];
    }
  }


  std::vector<pair_t> res = findPeakGrid();

  /*if (res.size() != 0) {
    Serial.println("New Readings:");
    for (int i = 0; i < res.size(); i++) {
      Serial.print("Peak ");
      Serial.print(i+1);
      Serial.print(": ");
      Serial.print(res[i].y);
      Serial.print(" ");
      Serial.println(res[i].x);
    }
    Serial.println();
  }*/
  
  assign_peaks(res);

  /*if (peaks.size() > 0 && peaks[0].active) {
    Serial.println("History of Peak 0:");
    for (int i = 0; i < peaks[0].pos_history.size(); i++) {
      Serial.print("(");
      Serial.print(peaks[0].pos_history[i].x);
      Serial.print(", ");
      Serial.print(peaks[0].pos_history[i].y);
      Serial.print(") ");
    }
    Serial.println();
  }*/

  count_people();

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
  
  bool response = myImager.setRangingFrequency(30);
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
      for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth)
      {
        for (int x = imageWidth-1; x >= 0; x--)
        {
          distance_curr[y/imageWidth][x] = measurementData.distance_mm[x+y];
        }
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
