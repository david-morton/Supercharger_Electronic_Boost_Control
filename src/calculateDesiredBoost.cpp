#include "calculateDesiredBoost.h"
#include "globalHelpers.h"

/* ======================================================================
   STRUCTURES: Holding boost by gear definitions
   ====================================================================== */
struct BoostByGear {
  int gear;
  int psi;
};

/* ======================================================================
   VARIABLES: General use / functional
   ====================================================================== */
float desiredBoostPsi = 0.0;

const BoostByGear boostByGearData[] = {
    {0, 0}, // First value is gear, second is boost maximum in PSI. Gear 0 is neutral
    {1, 1},
    {2, 2},
    {3, 3},
    {4, 4},
    {5, 5},
    {6, 6}};

/* ======================================================================
   FUNCTION: Determine desired boost level
   ====================================================================== */
float calculateDesiredBoostPsi(float speed, int rpm, int gear, bool clutchPressed) {
  DEBUG_BOOST("Calculating boost based on speed " + String(speed) + "kmh, rpm " + String(rpm) + ", gear " + String(gear) + " and clutch " + String(clutchPressed));
  if (speed <= 2 || gear == 0 || clutchPressed == true || rpm < 1000) {
    DEBUG_BOOST("Boost target set to 0psi due to conditional match (gear, speed, clutch etc)");
    return 0.0;
  } else {
    // Lookup boost by gear
    for (const auto &boostPair : boostByGearData) {
      if (boostPair.gear == gear) {
        DEBUG_BOOST("Boost target determined as " + String(boostPair.psi) + "psi");
        return boostPair.psi;
      }
    }
  }
  return 0.0;
  DEBUG_BOOST("Boost target set to 0psi as out of range gear provided");
}
