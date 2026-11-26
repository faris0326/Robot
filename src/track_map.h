#pragma once
// Track map (encoders needed). A Learn run records how sharply the robot
// turns, every MAP_SEG_MM of distance along the track; a Race run then plans
// a speed for every point: fast where it's straight, no faster than each
// curve allows, and braking early enough to get there (backward pass,
// v = sqrt(v_next^2 + 2 a d)). Speed control follows that plan; the steering
// stays reactive, so the map only sets the speed.
//
// Distance = the average of both wheels, so spinning on the spot adds about
// nothing. Turn ratio per segment = |dL - dR| / (|dL| + |dR|): 0 straight,
// 1 spinning on the spot. A Race run re-syncs its distance at every corner
// to the nearest corner in the map, so errors don't add up.
//
// Every call is from the control task (core 1), except the storage pair,
// which the telemetry task uses after mapSavePending().

#include <stdint.h>

void     mapLearnBegin(int32_t countL, int32_t countR);
void     mapLearnStep(int32_t countL, int32_t countR);   // every control tick of a Learn run
bool     mapLearnEnd();                                   // keeps it if long enough; true if kept
uint16_t mapLastLearnSegments();                          // length of the last Learn run, kept or not
float    mapLearnDistanceMm();                            // so far in the current / last Learn run
uint32_t mapLearnTicks();                                 // mapLearnStep() calls in it

void     mapRaceBegin(int32_t countL, int32_t countR, float topMms, float gripMms2);
float    mapRaceTarget(int32_t countL, int32_t countR, float speedMms);  // mm/s
void     mapRaceCorner(int32_t countL, int32_t countR);  // a corner just happened: re-sync

bool     mapReady();
uint16_t mapSegments();
uint16_t mapCorners();

// Storage: the turn ratios (0..255) of the current map. mapSavePending() is
// also set after a Learn run that was too short (nothing to save, but the
// phone should hear about it): check mapLastLearnKept().
bool           mapLastLearnKept();
bool           mapSavePending();
void           mapSaveDone();
const uint8_t* mapTurnData(uint16_t& count);
void           mapLoad(const uint8_t* turn, uint16_t count);  // at boot, before the control task
