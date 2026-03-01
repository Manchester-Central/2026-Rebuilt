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

- It's far simpler to use the SwerveDriveSimulation provided by MapleSim than to integrate the CTRE motor sims at the same time. The two mechanisms conflict, and there's no support by MapleSim to make this work at the moment. There are open tasks for someone brave at hear to try it.
- This should use a shared abstract class with the CTRE simulated and real drive train. This allows both code bases to share module data and robot metrics both drive systems will us.

# Gameification

## Audio

- Optional feature that was implemented for 2025, and has yet to return. It requires the addition of JavaFX which increases build and deployment time, which is why it's not integrated here.
