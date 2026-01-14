package frc.robot.subsystems;

import frc.robot.Constants;

public class Launcher {
    private ChaosTalonFx m_launcherMotor=new ChaosTalonFx(Constants.LauncherCanId, Constants.LauncherCanBus) ;

    public Launcher () {} 

    public void LauncherMotor100percent () {
        m_launerMotor.setspeed (1.0);
    }
}
