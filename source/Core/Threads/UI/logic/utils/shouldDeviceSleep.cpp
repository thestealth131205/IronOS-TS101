#include "Buttons.hpp"
#include "OperatingModeUtilities.h"
#include "power.hpp"

TickType_t        lastHallEffectSleepStart = 0;
extern TickType_t lastMovementTime;

// Minimum heater power draw (in 0.1W units, i.e. x10 Watts) above which Load Detection keeps
// the iron awake even without movement, e.g. while holding it still against a pad/trace to heat it.
static const uint32_t LoadDetectionThresholdX10Watts = 120; // 12.0W

bool shouldBeSleeping() {
#ifndef NO_SLEEP_MODE
  // Load Detection: if enabled and the heater is actively drawing significant power, do not enter
  // sleep due to inactivity (no movement/button presses), e.g. while holding the iron still
  // against a pad/trace to heat it up. Does not affect the hall-effect forced-sleep below, as
  // that reflects the iron being deliberately placed in its magnetic stand.
  bool loadDetected = getSettingValue(SettingsOptions::LoadDetection) && x10WattHistory.average() > LoadDetectionThresholdX10Watts;

  // Return true if the iron should be in sleep mode
  if (!loadDetected && getSettingValue(SettingsOptions::Sensitivity) && getSettingValue(SettingsOptions::SleepTime)) {
    // In auto start we are asleep until movement
    if (lastMovementTime == 0 && lastButtonTime == 0) {
      return true;
    }
    if (lastMovementTime > 0 || lastButtonTime > 0) {
      if (((xTaskGetTickCount() - lastMovementTime) > getSleepTimeout()) && ((xTaskGetTickCount() - lastButtonTime) > getSleepTimeout())) {
        return true;
      }
    }
  }

#ifdef HALL_SENSOR
  // If the hall effect sensor is enabled in the build, check if its over
  // threshold, and if so then we force sleep
  if (getHallSensorFitted() && lookupHallEffectThreshold()) {
    int16_t hallEffectStrength = getRawHallEffect();
    if (hallEffectStrength < 0) {
      hallEffectStrength = -hallEffectStrength;
    }
    // Have absolute value of measure of magnetic field strength
    if (hallEffectStrength > lookupHallEffectThreshold()) {
      if (lastHallEffectSleepStart == 0) {
        lastHallEffectSleepStart = xTaskGetTickCount();
      }
      if ((xTaskGetTickCount() - lastHallEffectSleepStart) > getHallEffectSleepTimeout()) {
        return true;
      }
    } else {
      lastHallEffectSleepStart = 0;
    }
  }
#endif
#endif
  return false;
}
