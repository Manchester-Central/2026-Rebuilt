package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.interfaces.IIntake;

public class DeployOuttake extends Command {
private IIntake m_intake;
public DeployOuttake(IIntake intake){
    m_intake = intake;
    addRequirements(intake);
}
@Override
public void initialize(){}

@Override
public void execute(){
    m_intake.setRollerSpeed(IntakeConstants.OuttakeRollerSpeed.get());
    m_intake.deploy();
} 

@Override
public boolean isFinished(){
    return false;
}

@Override
public void end(boolean interupted){}
}
