#include "calculateDesiredBoost.h"

/*
Define variables
*/
float desiredBoostPsi = 0.0;

/*
Define structure for holding boost by gear values
*/
struct BoostByGear {
  int gear;
  int psi;
};

/*
Define boost by gear lookup values
*/
const BoostByGear boostByGearData[] = {
    {0, 0}, // First value is gear, second is boost maximum in PSI
    {1, 2},
    {2, 4},
    {3, 8},
    {4, 8},
    {5, 8},
    {6, 8}};

/*
Define function - Calculate desired boost level
*/

float calculateDesiredBoostPsi(int gear, int speed, int rpm, bool clutchPressed) {
  if (speed <= 2 || gear == 0 || clutchPressed == true || rpm < 1000) {
    return 0.0;
  } else {
    // Calculate boost by gear
    for (const auto &boostPair : boostByGearData) {
      if (boostPair.gear == gear) {
        return boostPair.psi;
      }
    }
  }
  return desiredBoostPsi;
}

// TODO: Do we need to account for vacuum here when coasting down hill in gear say ? Should we look to capture pedal position ?
// Do we need to measure pressure in the charge piping as well as manifold ?
