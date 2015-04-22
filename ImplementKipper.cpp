/*
  ImplementKipper - a libary for a Kipper
 Copyright (C) 2011-2014 J.A. Woltjer.
 All rights reserved.
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ImplementKipper.h"

//------------
// Constructor
//------------

ImplementKipper::ImplementKipper(VehicleTractor * _tractor){
  // Pin configuration  
  // Outputs
  pinMode(OUTPUT_WIDE, OUTPUT);
  pinMode(OUTPUT_NARROW, OUTPUT);
  pinMode(OUTPUT_BYPASS, OUTPUT);
  pinMode(OUTPUT_LED, OUTPUT);

  // Analog IO
  analogReference(DEFAULT);
  pinMode(STEER_SENS_PIN, INPUT);
  digitalWrite(STEER_SENS_PIN, LOW);

  pinMode(ANGLE_SENS_PIN, INPUT);
  digitalWrite(ANGLE_SENS_PIN, LOW);

  // Get calibration data from EEPROM otherwise use defaults
  if (!readCalibrationData()){
    // Default steer calibration set 100, 101
    steer_calibration_data[0] = 201;
    steer_calibration_data[1] = 428;
    steer_calibration_data[2] = 687;

    // Default angle calibration set 110, 111
    angle_calibration_data[0] = 201;
    angle_calibration_data[1] = 428;
    angle_calibration_data[2] = 687;

    // PID constants 120, 121, 130, 131, 140, 141
    KP = 50;
    KI = 50;
    KD = 0;

    // offset 180
    offset = 0;
    
#ifdef DEBUG
    Serial.println("No calibration data found");
#endif
  }

  // Calibration points for steer and angle
  steer_calibration_points[0] = -30;
  steer_calibration_points[1] = 0;
  steer_calibration_points[2] = 30;
  steer = 0;
  last_steer = 0;

  angle_calibration_points[0] = -45;
  angle_calibration_points[1] =  0;
  angle_calibration_points[2] =  45;
  angle = 0;

  // Setpoint of ajust loop
  setpoint = 0;

  // PID integration and differentiation intervals
  for (int i = 0; i < 50; i++){
    angle_hist[i] = 0;
  }
  angle_sum = 0;      //Running sum of angle_hist
  angle_avg = 0;      //Average of sum

  dangle = 0;         //Dangle

  hist_count = 0;   //Counter of sum
  hist_time = 25;   //Integration time (seconds * 5)

  // PID variables
  P = 0;
  I = 0;
  D = 0;

  // End shutoff timers
  shutoff_time = SHUTOFF;
  shutoff_wide = false;
  shutoff_narrow = false;
  shutoff_timer = millis();

  // Update timer
  update_age = millis();
  
  // Print calibration data
#ifdef DEBUG
  printCalibrationData();
#endif
  tractor = _tractor;
}

// ----------------------------------
// Method for updating implement data
// ----------------------------------
void ImplementKipper::update(byte _mode){
  mode = _mode;

  // update offset, angle, steer and setpoint
  if (millis() - update_age >= 25){
    // Update angle, steer and setpoint
    if (update_flag){
      update_flag = false;
      
      // read angle, reset ad converter to steer input
      angle = getActualAngle();
      speed = tractor->getSpeedKmh();
      getActualSteer();
    }
    else {
      update_flag = true;
      
      // read steer, reset ad converter to angle input
      steer = getActualSteer();
      getActualAngle();
    }

    setSetpoint();
    update_age = millis();    
  }
}

// ------------------------------
// Method for adjusting implement
// ------------------------------
void ImplementKipper::adjust(int _direction){
  int _actual_steer;
  byte _pwm;

  if (mode == 0){
    _actual_steer = steer;
  }
  else {
    _actual_steer = setpoint - _direction;
  }

  // Adjust tree including error
  //---------------------------
  // Setpoint < actual steer
  //---------------------------
  if (_actual_steer < setpoint && !shutoff_narrow){
    digitalWrite(OUTPUT_WIDE, LOW);
    digitalWrite(OUTPUT_NARROW, HIGH);

    digitalWrite(OUTPUT_LED, HIGH);
    
    // End shutoff
    if (steer != last_steer){
      shutoff_timer = millis();
    }
    
    if (shutoff_wide){
      shutoff_narrow = false;
      shutoff_wide = false;
    }
    
    if (millis() - shutoff_timer > shutoff_time){
      shutoff_narrow = true;
      shutoff_wide = false;
    }
  }
  //---------------------------
  // Setpoint < actual steer
  //---------------------------
  else if (_actual_steer > setpoint && !shutoff_wide){
    digitalWrite(OUTPUT_WIDE, HIGH);
    digitalWrite(OUTPUT_NARROW, LOW);

    digitalWrite(OUTPUT_LED, HIGH);

    // End shutoff
    if (steer != last_steer){
      shutoff_timer = millis();
    }
    
    if (shutoff_narrow){
      shutoff_wide = false;
      shutoff_narrow = false;
    }
    
    if (millis() - shutoff_timer > shutoff_time){
      shutoff_narrow = false;
      shutoff_wide = true;
    }
  }
  //-----------------
  // Setpoint reached
  //-----------------
  else {
    digitalWrite(OUTPUT_WIDE, LOW);
    digitalWrite(OUTPUT_NARROW, LOW);

    digitalWrite(OUTPUT_BYPASS, LOW);

    digitalWrite(OUTPUT_LED, LOW);
    shutoff_timer = millis();
  }
  last_steer = steer;
}

// --------------------------------------------
// Method for measuring actual implement offset
// --------------------------------------------
int ImplementKipper::getActualSteer(){
  // Read analog input
  int _read_raw = analogRead(STEER_SENS_PIN);
  int _actual_steer;
  int i = 0;

  // Loop through calibrationdata
  while (_read_raw > steer_calibration_data[i] && i < 2){
    i++;
  }

  if (i == 0){
    i++;
  }

  // Interpolate calibrationdata
  float a = _read_raw - steer_calibration_data[i-1];
  float b = steer_calibration_data[i] - steer_calibration_data[i-1];
  float c = steer_calibration_points[i] - steer_calibration_points[i-1];
  float d = steer_calibration_points[i-1];

  // Calculate actual implement offset
  _actual_steer = (((a * c) / b) + d);

  return _actual_steer;
}

//------------------------------------------
// Method for measuring actual implement angle
//------------------------------------------
int ImplementKipper::getActualAngle(){
  // Read analog input
  int _read_raw = analogRead(ANGLE_SENS_PIN);
  int _actual_angle;
  int i = 0;

  // Loop through calibrationdata
  while (_read_raw > angle_calibration_data[i] && i < 2){
    i++;
  }

  if (i == 0){
    i++;
  }

  // Interpolate calibrationdata
  float a = _read_raw - angle_calibration_data[i-1];
  float b = angle_calibration_data[i] - angle_calibration_data[i-1];
  float c = angle_calibration_points[i] - angle_calibration_points[i-1];
  float d = angle_calibration_points[i-1];

  // Calculate actual implement offset
  _actual_angle = (((a * c) / b) + d);

  return _actual_angle;
}

// ---------------------------
// Method for setting setpoint
// ---------------------------
void ImplementKipper::setSetpoint(){
  int _previous_hist_count;
  int _angle_avg;
  int _dangle;
  int _dangle_calc;
  int _D_factor;
  int _angle_cor = angle + offset;

  _previous_hist_count= hist_count - 1;

  if (hist_count >= hist_time) {
    hist_count = 0;
    _previous_hist_count = hist_time - 1;
  }

  // Set angle sum , delta and averages;
  angle_sum = angle_sum - angle_hist[hist_count] + _angle_cor;
  _angle_avg = angle_sum / hist_time;
  _dangle = _angle_cor - angle_hist[_previous_hist_count];
  _dangle_calc = _angle_cor;//sqrt(abs(_angle))
  _D_factor = _dangle - _dangle_calc;

  // Update angle history
  angle_hist[hist_count] = _angle_cor;

  P = float(_angle_cor) * KP / 100.0f; 
  I = float(_angle_avg) * KI / 100.0f;
  D = D - (float(_D_factor) * KD / 100.0f) * speed;
  
  // Restrict D
  if (D > 15) {
    D = 15;
  }
  else if (D < -15) {
    D = -15;
  }

  // Reset D
  if (mode){
    D = 0;
  }

  setpoint = P + I + D;   

  hist_count++; 
}

// ----------------------------------------------
// Method for reading calibrationdata from EEPROM
// ----------------------------------------------
boolean ImplementKipper::readCalibrationData(){
  // Read amount of startups and add 1
  EEPROM.write(0, EEPROM.read(0) + 1);
  
  // Read offset and angle calibration data
  if (EEPROM.read(100) != 255 || EEPROM.read(110) != 255 ||
    EEPROM.read(120) != 255 || EEPROM.read(130) != 255 ||
    EEPROM.read(140) != 255 || EEPROM.read(150) != 255 ||
    EEPROM.read(160) != 255 || EEPROM.read(180) != 255){

    // Read from eeprom highbyte, then lowbyte, and combine into words
    for(int i = 0; i < 3; i++){
      int k = 2 * i;
      // 100 - 101 and 110 - 111
      steer_calibration_data[i] = word(EEPROM.read(k+100), EEPROM.read(k+101));
      angle_calibration_data[i] = word(EEPROM.read(k+110), EEPROM.read(k+111));
    }

    KP = EEPROM.read(120);
    KI = EEPROM.read(130);
    KD = EEPROM.read(140);

    
    if (EEPROM.read(180) < 255 || EEPROM.read(181) < 255){
      // Read offset (2 bytes)
      offset = word(EEPROM.read(180), EEPROM.read(181));
      if (offset > 20 || offset < -20){
        offset = 0;
      }
    }
    else {
      offset = 0;
    }
  }
  else {
    return false;
  }
  return true;
}

//---------------------------------------------------
//Method for printing calibration data to serial port
//---------------------------------------------------
void ImplementKipper::printCalibrationData(){
  // Print amount of times started
  Serial.println("Times started");
  Serial.println(EEPROM.read(0));
  Serial.println("-------------------------------");
  
  // Printing calibration data to serial port
  Serial.println("Using following data:");
  Serial.println("-------------------------------");
  Serial.println("Offset calibration data");
  for (int i = 0; i < 3; i++){
    Serial.print(steer_calibration_data[i]);
    Serial.print(", ");
    Serial.println(steer_calibration_points[i]);
  }
  Serial.println("-------------------------------");

#ifndef GPS
  Serial.println("angle calibration data");
  for (int i = 0; i < 3; i++){
    Serial.print(angle_calibration_data[i]);
    Serial.print(", ");
    Serial.println(angle_calibration_points[i]);
  }
  Serial.println("-------------------------------");
#endif
  
  Serial.println("KP");
  Serial.println(KP);
  Serial.println("-------------------------------");

  Serial.println("KI");
  Serial.println(KI);
  Serial.println("-------------------------------");

  Serial.println("KD");
  Serial.println(KD);
  Serial.println("-------------------------------");

}

// --------------------------------------------
// Method for writing calibrationdata to EEPROM
// --------------------------------------------
void ImplementKipper::writeCalibrationData(){
  // Write each byte separately to the memory first the data then the points
  for(int i = 0; i < 3; i++){
    int k = 2 * i;
    EEPROM.write(k+100, highByte(steer_calibration_data[i])); // 100, 102, 104
    EEPROM.write(k+101, lowByte(steer_calibration_data[i]));  // 101, 103, 105
    EEPROM.write(k+110, highByte(angle_calibration_data[i])); // 110, 112, 114
    EEPROM.write(k+111, lowByte(angle_calibration_data[i]));  // 111, 113, 115
  }

  EEPROM.write(120, KP); // 120
  EEPROM.write(130, KI); // 130
  EEPROM.write(140, KD); // 140
  
  EEPROM.write(180, highByte(offset)); // 180
  EEPROM.write(181, lowByte(offset)); // 181

#ifdef DEBUG
  Serial.println("Calibration data written");
#endif  
}

// ---------------------------------------------
// Method for wiping calibrationdata from EEPROM
// ---------------------------------------------
void ImplementKipper::wipeCalibrationData(){
  // Wipe calibration data

    // Write 255 into all memory registers
  for  (int i = 1; i < 255; i++){
    EEPROM.write(i, 255);
  }

#ifdef DEBUG
  Serial.println("Calibration data wiped");
#endif
}