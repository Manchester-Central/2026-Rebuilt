// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Add your docs here. */
public interface IHood extends Subsystem {
     /**
     * Sets the speed of the hood
     * @param speed double in the range of [-1.0, 1.0]
     */
    public void setHoodSpeed(double speed);

    /**
     * @return the current speed of the hood, in the range of [-1.0, 1.0]
     */
    public double getHoodSpeed();

    public void setHoodAngle(Angle targetAngle);

    public Angle getHoodAngle();
} 





