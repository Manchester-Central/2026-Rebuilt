package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class RetractIntake extends Command {
private Intake m_intake;
public RetractIntake(Intake intake){
    m_intake = intake;
    addRequirements(intake);
}
@Override
public void initialize(){}

@Override
public void execute(){
    m_intake.setRollerSpeed(0);
    m_intake.retract();
} 

@Override
public boolean isFinished(){
    return false;
}

@Override
public void end(boolean interupted){
    m_intake.setRollerSpeed(0);
    m_intake.setPivotSpeed(0);
}
}
