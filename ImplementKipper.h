/*
  ImplementKipper - a library for a kipper
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

#ifndef ImplementKipper_h
#define ImplementKipper_h

#include <Arduino.h>
#include <EEPROM.h>
#include "ConfigImplementKipper.h"
#include "VehicleGps.h"
#include "VehicleTractor.h"

// software version of this library
#define KIPPER_VERSION 0.1

class ImplementKipper {
private:
  //-------------
  // data members
  //-------------
  
  // Default angle calibration set
  int angle_calibration_data[3];
  int angle_calibration_points[3];
  int angle;

  // Default steer calibration set  
  int steer_calibration_data[3];
  int steer_calibration_points[3];
  int steer;
  int last_steer;
  
  int speed;

  // Update timer
  unsigned long update_age;
  boolean update_flag;

  // Variables concerning adjust loop
  byte mode;
  int setpoint;
  int offset;

  int angle_hist[50];
  int angle_sum;      //Running sum of xte_hist
  int angle_avg;      //Average of sum

  int dangle;         //DXTE

  byte hist_count;   //Counter of sum
  byte hist_time;   //Integration time (seconds * 5)

  // PID variables
  float P;
  byte KP;

  float I;
  byte KI;

  float D;
  byte KD;

  // Timers for end shutoff
  int shutoff_time;
  boolean shutoff_wide;
  boolean shutoff_narrow;
  unsigned long shutoff_timer;
  
  // Objects
  VehicleTractor * tractor;

  //-------------------------------------------------------------
  // private member functions implemented in ImplementKipper.cpp
  //-------------------------------------------------------------
  int getActualAngle();
  int getActualSteer();

  void setSetpoint();

  void readOffset();
  boolean readCalibrationData();
  void printCalibrationData();
  void writeCalibrationData();
  void wipeCalibrationData();

public:
  // -----------------------------------------------------------
  // public member functions implemented in ImplementKipper.cpp
  // -----------------------------------------------------------

  // Constructor
  ImplementKipper(VehicleTractor * _tractor);

  void update(byte _mode);
  void adjust(int _direction);
  void calibrate();

  // ----------------------------------------------------------------
  // public inline member functions implemented in ImplementKipper.h
  // ----------------------------------------------------------------
  inline boolean resetCalibration(){
    return readCalibrationData();
  }
  
  inline void commitCalibration(){
    wipeCalibrationData();
    writeCalibrationData();
  }
  
  // -------
  // Getters
  // -------
  inline int getSteer(){
    return steer;
  }
  
  inline int getAngle(){
    return angle;
  }
  
  inline int getSetpoint(){
    return setpoint;
  }
    
  inline int getSteerCalibrationPoint(int _i){
    return steer_calibration_points[_i];
  }

  inline int getAngleCalibrationPoint(int _i){
    return angle_calibration_points[_i];
  }

  inline byte getKP(){
    return KP;
  }

  inline byte getKI(){
    return KI;
  }

  inline byte getKD(){
    return KD;
  }

  inline int getOffset(){
    return offset;
  }

  // -------
  // Setters
  // -------  
  inline void setSteerCalibrationData(int _i){
    steer_calibration_data[_i] = analogRead(STEER_SENS_PIN);
  }

  inline void setAngleCalibrationData(int _i){
    angle_calibration_data[_i] = analogRead(ANGLE_SENS_PIN);
  }

  inline void setKP(byte _value){
    KP = _value;
  }

  inline void setKI(byte _value){
    KI = _value;
  }

  inline void setKD(byte _value){
    KD = _value;
  }

  inline void setOffset(int _value){
    offset = _value;
  }
};
#endif