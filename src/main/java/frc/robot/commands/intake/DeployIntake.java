package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.IntakeConstants.PivotConstants;
import frc.robot.constants.LauncherConstants.FeederConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

public class DeployIntake extends Command {
private Intake m_intake;
private Launcher m_launcher;

public DeployIntake(Intake intake, Launcher launcher){
    m_intake = intake;
    m_launcher = launcher;

    addRequirements(intake, launcher);
}
@Override
public void initialize(){}

@Override
public void execute(){
    if (m_intake.isNearAngle(PivotConstants.DeployAngle.get())) {
      m_intake.setRollerSpeed(IntakeConstants.IntakeRollerSpeed.get());   
    } else {
      m_intake.setRollerSpeed(IntakeConstants.DeployNetRollerSpeed.get());
    }

    m_launcher.setFeederSpeed(0, FeederConstants.OuttakeSpeed.get());
   
    m_intake.deploy();
} 

@Override
public boolean isFinished(){
    return false;
}

@Override
public void end(boolean interupted){
    m_intake.setRollerSpeed(0);

}
}
