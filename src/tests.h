#pragma once
// Bench tests, selected by the motortest / sensortest PlatformIO envs.

void motorTest();       // runs once: each motor both ways, spin, ramp
void sensorTestLoop();  // call from loop(): prints IR bits and position at 10 Hz
void steerTestLoop();   // call from loop(): runs control with motors live, prints L/R duty
void encoderTestBegin();  // instructions; motors stay braked
void encoderTestLoop();   // call from loop(): prints both encoder counts at 5 Hz
