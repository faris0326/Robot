#include "track_map.h"

#include <Arduino.h>
#include <string.h>

#include <atomic>

#include "config.h"

namespace {

const float MM_PER_COUNT = ENC_COUNTS_PER_REV > 0.0f ? PI * WHEEL_DIAMETER_MM / ENC_COUNTS_PER_REV : 0.0f;

uint8_t  turn[MAP_MAX_SEGMENTS];     // the map: turn ratio x 255 per segment
float    profile[MAP_MAX_SEGMENTS];  // planned speed per segment, mm/s (Race)
uint16_t segments = 0;
std::atomic<bool> savePending{false};
std::atomic<uint16_t> lastLearnSegments{0};
std::atomic<bool>     lastLearnKept{false};

// Learn run
uint8_t  learnTurn[MAP_MAX_SEGMENTS];
uint16_t learnCount = 0;
int32_t  lastL = 0, lastR = 0;
float    segMm = 0.0f;              // distance into the current segment
int32_t  sumAbs = 0, sumDiff = 0;   // wheel counts in the current segment
volatile uint32_t learnTicks = 0;   // for the phone: is the recorder running at all?

// Race run
int32_t startL = 0, startR = 0;
float   offsetMm = 0.0f;            // corner re-syncs

float turnRatio(uint16_t i) { return turn[i] / 255.0f; }

float distanceMm(int32_t countL, int32_t countR) {
    return ((countL - startL) + (countR - startR)) * 0.5f * MM_PER_COUNT + offsetMm;
}

}  // namespace

void mapLearnBegin(int32_t countL, int32_t countR) {
    learnCount = 0;
    lastL = countL;
    lastR = countR;
    segMm = 0.0f;
    sumAbs = sumDiff = 0;
    learnTicks = 0;
}

void mapLearnStep(int32_t countL, int32_t countR) {
    learnTicks = learnTicks + 1;
    int32_t dL = countL - lastL, dR = countR - lastR;
    lastL = countL;
    lastR = countR;
    sumAbs += abs(dL) + abs(dR);
    sumDiff += dL - dR;
    segMm += (dL + dR) * 0.5f * MM_PER_COUNT;
    while (segMm >= MAP_SEG_MM && learnCount < MAP_MAX_SEGMENTS) {
        float t = sumAbs > 0 ? min(1.0f, (float)abs(sumDiff) / sumAbs) : 0.0f;
        learnTurn[learnCount++] = (uint8_t)lroundf(t * 255.0f);
        segMm -= MAP_SEG_MM;
        sumAbs = sumDiff = 0;
    }
}

bool mapLearnEnd() {
    lastLearnSegments = learnCount;
    lastLearnKept = learnCount >= MAP_MIN_SEGMENTS;
    if (lastLearnKept) {
        memcpy(turn, learnTurn, learnCount);
        segments = learnCount;
    }
    savePending = true;  // tell the phone either way
    return lastLearnKept;
}

void mapRaceBegin(int32_t countL, int32_t countR, float topMms, float gripMms2) {
    startL = countL;
    startR = countR;
    offsetMm = 0.0f;
    if (segments == 0) return;

    // Each segment's own limit: straight = top speed; a curve of radius R
    // allows sqrt(grip x R). The turn ratio is averaged over 3 segments so a
    // wobble on a straight doesn't count as a curve. R = half the wheel track
    // / turn ratio (pivoting on the spot: about the half track).
    for (uint16_t i = 0; i < segments; i++) {
        float t = turnRatio(i);
        int n = 1;
        if (i > 0) { t += turnRatio(i - 1); n++; }
        if (i + 1 < segments) { t += turnRatio(i + 1); n++; }
        t /= n;
        float v = topMms;
        if (t >= MAP_STRAIGHT_TURN) v = sqrtf(gripMms2 * (WHEEL_TRACK_MM * 0.5f) / t);
        // A corner (the robot spun on the spot there) is never averaged away:
        // it needs the robot nearly stopped.
        if (turnRatio(i) >= MAP_CORNER_TURN) v = MAP_MIN_MMS;
        profile[i] = constrain(v, MAP_MIN_MMS, topMms);
    }
    // Brake in time: no faster than what can still slow down to the next segment.
    for (int i = segments - 2; i >= 0; i--) {
        float reach = sqrtf(profile[i + 1] * profile[i + 1] + 2.0f * gripMms2 * MAP_SEG_MM);
        profile[i] = min(profile[i], reach);
    }
}

float mapRaceTarget(int32_t countL, int32_t countR, float speedMms) {
    if (segments == 0) return MAP_MIN_MMS;
    float ahead = distanceMm(countL, countR) + max(speedMms, 0.0f) * MAP_LAG_S;
    int i = (int)(ahead / MAP_SEG_MM);
    if (i < 0) i = 0;
    if (i >= segments) i = segments - 1;   // past the end: the last planned speed
    return profile[i];
}

void mapRaceCorner(int32_t countL, int32_t countR) {
    float d = distanceMm(countL, countR);
    float best = MAP_SYNC_MM;
    float shift = 0.0f;
    for (uint16_t i = 0; i < segments; i++) {
        if (turnRatio(i) < MAP_CORNER_TURN) continue;
        float centre = (i + 0.5f) * MAP_SEG_MM;
        if (fabsf(centre - d) < best) {
            best = fabsf(centre - d);
            shift = centre - d;
        }
    }
    offsetMm += shift;
}

bool     mapReady() { return segments > 0; }
uint16_t mapSegments() { return segments; }

uint16_t mapCorners() {
    uint16_t n = 0;
    for (uint16_t i = 0; i < segments; i++) {
        bool corner = turnRatio(i) >= MAP_CORNER_TURN;
        if (corner && (i == 0 || turnRatio(i - 1) < MAP_CORNER_TURN)) n++;
    }
    return n;
}

uint16_t       mapLastLearnSegments() { return lastLearnSegments; }
float          mapLearnDistanceMm() { return learnCount * MAP_SEG_MM + segMm; }
uint32_t       mapLearnTicks() { return learnTicks; }
bool           mapLastLearnKept() { return lastLearnKept; }
bool           mapSavePending() { return savePending; }
void           mapSaveDone() { savePending = false; }
const uint8_t* mapTurnData(uint16_t& count) { count = segments; return turn; }

void mapLoad(const uint8_t* t, uint16_t count) {
    segments = min<uint16_t>(count, MAP_MAX_SEGMENTS);
    memcpy(turn, t, segments);
}
