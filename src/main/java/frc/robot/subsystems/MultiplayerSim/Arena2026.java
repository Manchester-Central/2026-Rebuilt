package frc.robot.subsystems.MultiplayerSim;

import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Arena2026 extends Arena2026Rebuilt {
  public Arena2026(boolean b) {
    super(b);
  }

  public void setFirstTeam(Alliance team) {
    blueIsOnClock = team == Alliance.Blue;
  }
}
