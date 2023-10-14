// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.simulations.SingleJointedArm;

/** Moves the arm simulation to a setpoint */
public class SingledJointedCommand extends CommandBase
{
    private SingleJointedArm m_SingleJointedArm;

    private double m_setpoint;

    public SingledJointedCommand(SingleJointedArm singleJointedArm, double setpoint) 
    {
        m_SingleJointedArm = singleJointedArm;
        m_setpoint = setpoint;

        addRequirements(singleJointedArm);
    }

    @Override
    public void execute() 
    {
        m_SingleJointedArm.armToSetpoint(m_setpoint);
    }

    @Override
    public void end(boolean interrupted)
    {
        // m_SingleJointedArm.stopArmMotor();
    }
}
