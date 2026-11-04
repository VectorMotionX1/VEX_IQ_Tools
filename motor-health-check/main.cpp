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

// IQ Gen 2 Motor Health Check
// Bench test: secure the motors and give each shaft the same load.
motor m1(PORT1), m2(PORT2), m3(PORT3), m4(PORT4), m5(PORT5), m6(PORT6);
motor* const motors[] = {&m1, &m2, &m3, &m4, &m5, &m6};
const int speeds[] = {30, 60, 90};

struct Reading {
  double rpm, amps, temp, efficiency, angle;
};
Reading readings[6][3] = {};
enum Status { Ready, Missing, Hot, Slow, Outlier };
Status status[6] = {};
const char* const labels[] = {"OK", "MISSING", "HOT", "LOW RPM", "OUTLIER"};

template <typename... Args>
void line(int row, const char* format, Args... args) {
  Brain.Screen.setCursor(row, 1);
  Brain.Screen.print(format, args...);
}

bool measure(int port, int stage) {
  motor& motorUnderTest = *motors[port];
  Reading& result = readings[port][stage];
  if (!motorUnderTest.installed()) {
    status[port] = Missing;
    return false;
  }
  if (motorUnderTest.temperature(celsius) >= 55) {
    status[port] = Hot;
    return false;
  }
  Brain.Screen.clearScreen();
  line(1, "Testing port %d", port + 1);
  line(2, "Target %d%%", speeds[stage]);
  const double start = motorUnderTest.position(degrees);
  motorUnderTest.spin(forward, speeds[stage], velocityUnits::pct);
  // Allow 600 ms to settle, then average 15 readings over 1.5 s.
  for (int i = 0; i < 21; ++i) {
    wait(100, msec);
    if (!motorUnderTest.installed()) {
      status[port] = Missing;
      motorUnderTest.stop(coast);
      return false;
    }
    const double temp = motorUnderTest.temperature(celsius);
    if (temp >= 55) {
      status[port] = Hot;
      motorUnderTest.stop(coast);
      return false;
    }
    if (i < 6) continue;
    result.rpm += motorUnderTest.velocity(rpm) / 15.0;
    result.amps += motorUnderTest.current(amp) / 15.0;
    result.efficiency += motorUnderTest.efficiency(percent) / 15.0;
    if (temp > result.temp) result.temp = temp;
  }
  result.angle = motorUnderTest.position(degrees) - start;
  motorUnderTest.stop(coast);
  wait(500, msec);
  return true;
}

int main() {
  vexcodeInit();
  Brain.Screen.setFont(mono12);
  Brain.Screen.clearScreen();
  line(1, "Motor Health Check");
  line(2, "Secure motors / lift");
  line(3, "wheels. Same load.");
  line(4, "Press RIGHT to start");
  while (!Brain.buttonRight.pressing()) wait(20, msec);
  while (Brain.buttonRight.pressing()) wait(20, msec);

  int completed[6] = {};
  for (int port = 0; port < 6; ++port) {
    for (int stage = 0; stage < 3; ++stage) {
      if (!measure(port, stage)) break;
      ++completed[port];
      // Screening floor: 80% of the commanded fraction of 120 RPM.
      if (readings[port][stage].rpm < 0.96 * speeds[stage])
        status[port] = Slow;
    }
  }

  // Build peer medians before assigning any outlier flags.
  bool outlier[6] = {};
  for (int stage = 0; stage < 3; ++stage) {
    double speed[6], current[6];
    int count = 0;
    for (int port = 0; port < 6; ++port) {
      if (status[port] != Ready) continue;
      speed[count] = readings[port][stage].rpm;
      current[count] = readings[port][stage].amps;
      ++count;
    }
    if (count < 3) continue;
    for (int i = 0; i < count; ++i) {
      for (int j = i + 1; j < count; ++j) {
        if (speed[j] < speed[i]) {
          double value = speed[i]; speed[i] = speed[j]; speed[j] = value;
        }
        if (current[j] < current[i]) {
          double value = current[i]; current[i] = current[j]; current[j] = value;
        }
      }
    }
    double medianRpm = (speed[(count - 1) / 2] + speed[count / 2]) / 2;
    double medianAmps = (current[(count - 1) / 2] + current[count / 2]) / 2;
    for (int port = 0; port < 6; ++port) {
      if (status[port] != Ready) continue;
      const Reading& result = readings[port][stage];
      if (fabs(result.rpm - medianRpm) > 0.08 * medianRpm ||
          (result.amps > 1.5 * medianAmps && result.amps > medianAmps + 0.10))
        outlier[port] = true;
    }
  }
  for (int port = 0; port < 6; ++port)
    if (outlier[port]) status[port] = Outlier;

  int best[4] = {};
  double bestSpread = 1e9;
  double bestPairGap = 1e9;
  for (int mask = 1; mask < 64; ++mask) {
    int ports[6], count = 0;
    for (int port = 0; port < 6; ++port)
      if ((mask & (1 << port)) && status[port] == Ready) ports[count++] = port;
    if (count != 4 || mask != ((1 << ports[0]) | (1 << ports[1]) |
                              (1 << ports[2]) | (1 << ports[3]))) continue;

    double spread = 0;
    for (int stage = 0; stage < 3; ++stage) {
      double low = readings[ports[0]][stage].rpm, high = low;
      for (int i = 1; i < 4; ++i) {
        const double speed = readings[ports[i]][stage].rpm;
        if (speed < low) low = speed;
        if (speed > high) high = speed;
      }
      spread += (high - low) / (1.2 * speeds[stage]);
    }
    // Of the three possible splits, balance the two sides across all speeds.
    for (int mate = 1; mate < 4; ++mate) {
      int other[2], next = 0;
      for (int i = 1; i < 4; ++i)
        if (i != mate) other[next++] = ports[i];
      double pairGap = 0;
      for (int stage = 0; stage < 3; ++stage) {
        pairGap += fabs(readings[ports[0]][stage].rpm +
                        readings[ports[mate]][stage].rpm -
                        readings[other[0]][stage].rpm -
                        readings[other[1]][stage].rpm) / (2.4 * speeds[stage]);
      }
      if (spread < bestSpread - 1e-9 ||
          (fabs(spread - bestSpread) < 1e-9 && pairGap < bestPairGap)) {
        bestSpread = spread;
        bestPairGap = pairGap;
        best[0] = ports[0] + 1;
        best[1] = ports[mate] + 1;
        best[2] = other[0] + 1;
        best[3] = other[1] + 1;
      }
    }
  }

  // Summary plus 18 pages, one per port and speed.
  int page = 0;
  while (true) {
    Brain.Screen.clearScreen();
    if (page == 0) {
      line(1, "Best drivetrain set");
      if (best[0]) {
        line(2, "Left %d,%d Right %d,%d", best[0], best[1], best[2], best[3]);
        line(3, "Mean RPM span %.1f%%", bestSpread * 100 / 3);
      } else {
        line(2, "Fewer than 4 passed");
      }
      line(4, "Flags:%s%s%s%s%s%s",
           status[0] ? " 1" : "", status[1] ? " 2" : "",
           status[2] ? " 3" : "", status[3] ? " 4" : "",
           status[4] ? " 5" : "", status[5] ? " 6" : "");
    } else {
      const int port = (page - 1) / 3, stage = (page - 1) % 3;
      line(1, "P%d %d%% %s", port + 1, speeds[stage], labels[status[port]]);
      if (stage < completed[port]) {
        const Reading& result = readings[port][stage];
        line(2, "%.1f RPM  %.2f A", result.rpm, result.amps);
        line(3, "%.1f C  Eff %.1f%%", result.temp, result.efficiency);
        line(4, "Angle %.0f deg", result.angle);
      } else {
        line(2, "No complete reading");
      }
    }
    line(5, "< Prev   %02d   Next >", page);
    while (!Brain.buttonLeft.pressing() && !Brain.buttonRight.pressing())
      wait(20, msec);
    page = (page + (Brain.buttonLeft.pressing() ? 18 : 1)) % 19;
    while (Brain.buttonLeft.pressing() || Brain.buttonRight.pressing())
      wait(20, msec);
  }
}
