// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.simulations.Elevator;

/** Moves the elevator simulation to a setpoint */
public class ElevatorCommand extends CommandBase
{
    private Elevator m_elevator;

    private double m_setpoint;

    public ElevatorCommand(Elevator elevator, double setpoint) 
    {
        m_elevator = elevator;
        m_setpoint = setpoint;

        addRequirements(elevator);
    }

    @Override
    public void execute() 
    {
        m_elevator.elevatorToSetpoint(m_setpoint);
    }

    @Override
    public void end(boolean interrupted) {}
}
