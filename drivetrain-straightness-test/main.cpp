#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START IQ MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END IQ MACROS


// Robot configuration code.
inertial BrainInertial = inertial();


// generating and setting random seed
void initializeRandomSeed(){
  wait(100,msec);
  double xAxis = BrainInertial.acceleration(xaxis) * 1000;
  double yAxis = BrainInertial.acceleration(yaxis) * 1000;
  double zAxis = BrainInertial.acceleration(zaxis) * 1000;
  // Combine these values into a single integer
  int seed = int(
    xAxis + yAxis + zAxis
  );
  // Set the seed
  srand(seed); 
}



void vexcodeInit() {

  // Initializing random seed.
  initializeRandomSeed(); 
}

#pragma endregion VEXcode Generated Robot Configuration

//----------------------------------------------------------------------------
//                                                                            
//    Module:       main.cpp                                                  
//    Author:       {author}                                                  
//    Created:      {date}                                                    
//    Description:  IQ project                                                
//                                                                            
//----------------------------------------------------------------------------

// Include the IQ Library
#include "iq_cpp.h"

// Allows for easier use of the VEX Library
using namespace vex;

// VEX IQ Gen 2 four-motor straightness test.
// Change these constants to match the actual drivetrain before testing.
// Measure wheel travel per revolution under normal load if possible.
const double wheelTravelMm = 200.0;
const double motorTurnsPerWheelTurn = 1.0;
const double targetMm = 1000.0;
const int speedPct = 60;
const int ports[] = {1, 3, 2, 6};

motor left1(PORT1, false), left3(PORT3, false);
motor right2(PORT2, true), right6(PORT6, true);
motor* const drive[] = {&left1, &left3, &right2, &right6};

struct Trial {
  double startYaw, cruiseYaw, preStopYaw, finalYaw, peakYaw, distanceMm;
  double angle[4], amps[4];
};

template <typename... Args>
void show(int row, const char* format, Args... args) {
  Brain.Screen.setCursor(row, 1);
  Brain.Screen.print(format, args...);
}

bool waitForRight() {
  while (Brain.buttonLeft.pressing() || Brain.buttonRight.pressing())
    wait(20, msec);
  while (!Brain.buttonLeft.pressing() && !Brain.buttonRight.pressing())
    wait(20, msec);
  const bool proceed = !Brain.buttonLeft.pressing();
  while (Brain.buttonLeft.pressing() || Brain.buttonRight.pressing())
    wait(20, msec);
  return proceed;
}

bool connected() {
  for (int i = 0; i < 4; ++i) {
    if (!drive[i]->installed()) {
      Brain.Screen.clearScreen();
      show(1, "Missing motor P%d", ports[i]);
      return false;
    }
  }
  return true;
}

double encoderDistanceMm(const double zero[4]) {
  double sum = 0;
  for (int i = 0; i < 4; ++i)
    sum += drive[i]->position(degrees) - zero[i];
  return sum * wheelTravelMm / (4 * 360 * motorTurnsPerWheelTurn);
}

bool runTrial(Trial& result) {
  if (!connected()) return false;
  double zero[4];
  for (int i = 0; i < 4; ++i) zero[i] = drive[i]->position(degrees);
  const double yawZero = BrainInertial.rotation(degrees);
  timer elapsed;
  elapsed.reset();
  for (auto* m : drive) m->spin(forward, speedPct, velocityUnits::pct);

  bool startCaptured = false, cruiseCaptured = false;
  int samples = 0;
  const char* error = nullptr;
  double traveled = 0;
  while (true) {
    const double yaw = BrainInertial.rotation(degrees) - yawZero;
    traveled = encoderDistanceMm(zero);
    if (fabs(yaw) > fabs(result.peakYaw)) result.peakYaw = yaw;
    if (!startCaptured && traveled >= 0.20 * targetMm) {
      result.startYaw = yaw;
      startCaptured = true;
    }
    if (!cruiseCaptured && traveled >= 0.70 * targetMm) {
      result.cruiseYaw = yaw;
      cruiseCaptured = true;
    }
    if (Brain.buttonLeft.pressing()) error = "User stop";
    else if (!connected()) error = "Motor disconnected";
    else if (fabs(yaw) > 20) error = "Yaw exceeded 20 deg";
    else if (elapsed.time(msec) >= 7000) error = "Distance timeout";
    else if (elapsed.time(msec) >= 750 && traveled < 25) error = "Check drive direction";
    if (error || traveled >= targetMm) {
      result.preStopYaw = yaw;
      break;
    }
    for (int i = 0; i < 4; ++i) result.amps[i] += drive[i]->current(amp);
    ++samples;
    wait(25, msec);
  }
  for (auto* m : drive) m->stop(brake);
  if (error) {
    Brain.Screen.clearScreen();
    show(1, "%s", error);
    show(2, "Run aborted");
    return false;
  }
  wait(350, msec);
  result.finalYaw = BrainInertial.rotation(degrees) - yawZero;
  result.distanceMm = encoderDistanceMm(zero);
  for (int i = 0; i < 4; ++i) {
    result.angle[i] = drive[i]->position(degrees) - zero[i];
    result.amps[i] /= samples > 0 ? samples : 1;
  }
  return true;
}

void trialPages(const Trial& trial, int number) {
  int page = 0;
  while (true) {
    Brain.Screen.clearScreen();
    if (page == 0) {
      show(1, "Run %d/6  %c  %.0fmm", number, number <= 3 ? 'A' : 'B',
           trial.distanceMm);
      show(2, "Yaw 20%% %+.1f deg", trial.startYaw);
      show(3, "Yaw 70%% %+.1f deg", trial.cruiseYaw);
      show(4, "Brake %+.1f End %+.1f", trial.preStopYaw, trial.finalYaw);
      show(5, "LEFT motors  RIGHT done");
    } else {
      for (int i = 0; i < 4; ++i)
        show(i + 1, "P%d %6.0fdeg %.2fA", ports[i], trial.angle[i], trial.amps[i]);
      show(5, "LEFT yaw  RIGHT done");
    }
    if (waitForRight()) return;
    page = 1 - page;
  }
}

int main() {
  vexcodeInit();
  Brain.Screen.setFont(mono12);
  if (!connected()) return 0;
  Brain.Screen.clearScreen();
  show(1, "Straightness Test");
  show(2, "Lift wheels first");
  show(3, "RIGHT: direction test");
  show(4, "LEFT: exit");
  if (!waitForRight() || !connected()) return 0;

  for (auto* m : drive) m->spin(forward, 20, velocityUnits::pct);
  for (int i = 0; i < 30 && !Brain.buttonLeft.pressing(); ++i) wait(25, msec);
  for (auto* m : drive) m->stop(brake);
  if (Brain.buttonLeft.pressing()) return 0;

  Brain.Screen.clearScreen();
  show(1, "All wheels forward?");
  show(2, "Place car on start");
  show(3, "RIGHT yes / LEFT exit");
  if (!waitForRight()) return 0;

  Brain.Screen.clearScreen();
  show(1, "Calibrating gyro");
  show(2, "Keep robot still");
  BrainInertial.calibrate();
  timer calibration;
  while (BrainInertial.isCalibrating() && calibration.time(msec) < 5000)
    wait(20, msec);
  if (BrainInertial.isCalibrating()) {
    Brain.Screen.clearScreen();
    show(1, "Gyro calibration failed");
    return 0;
  }

  double yawSum[2] = {}, yawAbsSum[2] = {}, maxFinal = 0, maxDuring = 0;
  for (int run = 0; run < 6; ++run) {
    Brain.Screen.clearScreen();
    show(1, "Ready %c%d  %d%%  1m", run < 3 ? 'A' : 'B', run % 3 + 1, speedPct);
    show(2, run == 3 ? "Turn robot 180 deg" : "Align on start line");
    show(3, "Clear path. RIGHT go");
    show(4, "LEFT exits or stops");
    if (!waitForRight()) return 0;

    Trial trial = {};
    if (!runTrial(trial)) return 0;
    yawSum[run / 3] += trial.finalYaw;
    yawAbsSum[run / 3] += fabs(trial.finalYaw);
    if (fabs(trial.finalYaw) > maxFinal) maxFinal = fabs(trial.finalYaw);
    if (fabs(trial.peakYaw) > maxDuring) maxDuring = fabs(trial.peakYaw);
    trialPages(trial, run + 1);
  }
  Brain.Screen.clearScreen();
  show(1, "End yaw: mean / abs");
  show(2, "A %+.1f / %.1f deg", yawSum[0] / 3, yawAbsSum[0] / 3);
  show(3, "B %+.1f / %.1f deg", yawSum[1] / 3, yawAbsSum[1] / 3);
  show(4, "Worst end %.1f deg", maxFinal);
  show(5, "Peak moving %.1f deg", maxDuring);
  while (true) wait(50, msec);
}
