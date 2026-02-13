package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.interfaces.IIntake;

public class IntakeCommand extends Command {
private IIntake intake;
public IntakeCommand(IIntake intake){
    this.intake=intake;
    addRequirements(intake);
}
@Override
public void initialize(){}

@Override
public void execute(){
    intake.setRollerSpeed(IntakeConstants.RollerSpeed.get());
    intake.deploy();
} 

@Override
public boolean isFinished(){
    return false;
}

@Override
public void end(boolean interupted){}
}
