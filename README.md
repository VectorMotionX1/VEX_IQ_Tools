# VEX IQ Tools

Two C++ diagnostics for the VEX IQ (2nd generation) Brain. Each tool has its own folder.

| Tool | Use it to |
| --- | --- |
| [Motor Health Check](motor-health-check/main.cpp) | Compare up to six motors and choose four with closely matched speeds. |
| [Drivetrain Straightness Test](drivetrain-straightness-test/main.cpp) | Measure yaw under real driving load and compare changes to wheels, motors, or chassis setup. |

## Load a tool

1. Open [VEXcode IQ](https://codeiq.vex.com/) and choose **File → New Text Project → C++** for a second-generation Brain.
2. Open the desired `main.cpp` above, copy the raw source, and replace the contents of the editor in the new project. The source includes the VEXcode configuration. Set the ports in the source.
3. Save the VEXcode project, connect the Brain, choose an available slot, and select **Download**. Run the project from the Brain.

## Motor Health Check

1. Plug the motors into ports **1–6**. Secure the motors so each shaft can spin freely under the same load. Label the physical motors by test port before moving them.
2. Run the program and press the Brain's **Right** button. It tests one motor at a time at **30%, 60%, and 90%**. A full run takes about **47 seconds**.
3. Read **Best drivetrain set** for the four recommended ports and the suggested left/right pairs. The group minimizes the average normalized RPM spread across the three speeds. The pairing minimizes the difference between the two sides' mean RPM across those speeds.
4. Use **Left/Right** to page through RPM, average current, peak sampled temperature, efficiency, and shaft rotation. **Flags** lists ports needing review. Missing, hot, low-RPM, and outlier motors are excluded from the recommendation. If fewer than four pass, the screen says so.

**Mean RPM span** is the average, over the three speeds, of the gap between the group's fastest and slowest motors divided by that stage's target RPM. A low value indicates close matching under the test load.

## Drivetrain Straightness Test

| Side | Default Brain ports |
| --- | --- |
| Left | 3 and 5 |
| Right | 9 and 11 |

1. Check the port list and motor definitions near the top of the source. Distance defaults to **200 mm wheel travel per revolution** and **1 motor turn per wheel turn**. Set these to match your drivetrain. The default test is **60% speed over an estimated 1,000 mm**.
2. Run the program, lift the wheels off the ground, and press **Right** for a brief low-speed direction check. Confirm all four wheels would drive the robot forward. If a wheel is wrong, press **Left** to exit and correct its reversal setting. Put the robot on the start line and keep it still during gyro calibration.
3. Run **A1–A3** along the same marked line. Before **B1**, turn the robot **180°** and run **B1–B3** along that line in the opposite direction. Align it before each run and clear the path. **Right** starts each run. **Left** stops an active run.
4. After each run, read yaw at **20%**, **70%**, just before braking, and after stopping. **Left** toggles the motor page with encoder angles and mean current. **Right** advances. Measure the signed sideways distance from the marked line to the same reference point on the robot at the finish.

**End yaw: mean / abs** shows the signed mean and the mean absolute settled yaw for each direction. Negative yaw is left and positive yaw is right, relative to the robot's starting direction. **Worst end** is the largest absolute settled yaw. **Peak moving** is the largest absolute yaw sampled before braking. The Brain uses the gyro for measurement and a large-yaw abort. The test sends equal speed commands to both sides throughout the run.

When the robot consistently veers toward its own same side in both directions, swap only the physical motors left-to-right and repeat the same test. If the bias stays on the same side, compare wheel seating, rolling circumference, and axle alignment. Swap only one wheel pair at a time to isolate the change. Use both heading and measured sideways distance to compare setups.
