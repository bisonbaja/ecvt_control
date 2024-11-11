// Main file for Baja CVT Controller
// Program name saddle 
// Christian Wilson 11/8/24

#include <Arduino.h>
#include <SD.h>
#include <CAN.h>
#define ENGINE_TACH_PIN     A1
#define SECONDARY_TACH_PIN  A2
#define TACH_SCALE          2475/128

// Control EQN:
// P = 0.75 - (Ka / (Rt+1)) + Kp E + Kd dE/dt + Ki integ(E)
// Exlicit term, proportional, derivative, integral

double erpmMeasured;
double erpmTarget;
double srpmMeasured;
double ratioTarget;
double ratioMeasured;

void update() {

}