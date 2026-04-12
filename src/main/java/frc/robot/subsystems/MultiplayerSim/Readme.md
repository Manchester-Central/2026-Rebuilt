# Usage

## Elastic

This is required to control the simulation, it leverages many of the simulation gui mechanisms, but does so from its own control scheme. The issue is in interface simplicity, and it's easier to configure/control via Elastic than any other interface.

Open up Elastic and add multiple widgets to the panel:

| Arena/StartMatch | A button to start a match |
| Arena/AbortMatch | A button to stop a match in its current state |
| Robot#/Enabled | Toggle to enable or disable that robot |
| Robot#/AutoChoice | A dropdown to select an auto for that robot to run |

There are several optional widgets that can be added:

| Arena/MatchActive | Red/Green indicator that as match thread is running |
| Arena/MatchState | The name of the match phase (enum) |
| Arena/RedScore | Score for the red team |
| Arena/RedActive | If the red hub is active |
| Arena/BlueScore | Score for the blue team |
| Arena/BlueActive | If the blue hub is active |

### Alternatively...

Add the included "Multiplayer" tab in the elastic-tab.json file into your teams elastic layout config. This should be placed inside the json `tabs` array. This comes pre-configured with all necessary auto choosers, indicators, and match buttons.

## AdvantageScope

The simulation is handled within a robot loop, and the best way to display the simulation state/output is via AdvantageScope.

Make sure to do this in a distinct 3D Field tab to not interfere with normal robot workflows. Add the following fields onto the Poses section at the bottom.

| Robot#/Pose2d | Robot's pose on the field |
| Robot#/Mech3d | Animated representation of the robot's moving mechanisms - Drop ONTO corresponding Pose2d, it will look indented if done correctly |

### Alternatively...

Add the included "Arena" tab in the advantagescope-tab.json file into your teams AdvantageScope layout config. This should be placed inside the json `hubs[0]['state']['tabs']['tabs']` array. This comes pre-configured with all the necessary pose references, mechanism attachments, and match items and game pieces.

# Integration

This is the documentation for how MultiplayerArena was integrated into this year's robot code.

## Robot.java

- Marry the ARENA robot mode with the SIM robot mode when setting up the Logger data receivers and writers.
- Right before or after setting up the standard robot container, run `MultiplayerArena2026.Instance.loadAdditionalRobots()`.
- To restart a match, run `MultiplayerArena2026.Instance.prepareMatch()`. This is typically done during `disabledInit()`.
- The simulation need to be run in a periodic loop. This should not be done during `simulationPeriodic()` because momentum can linger in the simulation and lead to edge cases. Instead, it should be called during `teleopPeriodic()` or `autonomousPeriodic()`.
- A match needs to be started via `MultiplayerArena2026.Instance.startMatch()`, this should likewise be started when players can control the robot (`teleopInit()`), or autonomous routines can be executed (`autonomousInit()`).
- `MultiplayerArena2026` does not (currently) check for the current robot mode and calls to start matches, or run the periodic call, should check the robot mode is `ARENA` before executing.

## RobotContainer.java

- All robot containers need to include an index to determine which robot is being setup. Mostly this has to do with motor ID mapping (because CTRE doesn't simulate multiple buses), and controller setup. 

## Robot Number

- There needs to be some way to distinguish robots from each other. So each robot is given a number. 0 is the primary robot, and 0-2 are the blue alliance. 3-5 are the red alliance. As a result, you can see the value 0 tucked into non-arena constructors, and these can be discarded in those situations.
- In order to support multiple robots, robot logging should be placed and named with that in mind. Current convention is to place a `"Robot"+id+"/"` in front of mechanisms and data to indicate which mechanism corresponds to which mechanism. During any other robot mode, all logs would still show up under `Robot0`.

## Swerve Drive

- It's far simpler to use the SwerveDriveSimulation provided by MapleSim than to integrate the CTRE motor sims at the same time. The two mechanisms conflict, and there's no support by MapleSim to make this work at the moment. There are open tasks for someone brave at heart to try it.
- This should use a shared abstract class with the CTRE simulated and real drive train. This allows both code bases to share module data and robot metrics both drive systems will us.
