// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ElevatorCommand;
import frc.robot.simulations.Elevator;

public class RobotContainer 
{
    private CommandXboxController m_pilotController = new CommandXboxController(0);

    private Elevator m_elevator = new Elevator();

	public RobotContainer() 
	{
		configureBindings();
	}

	private void configureBindings() 
    {
        m_pilotController.a()
            .whileTrue(new ElevatorCommand(m_elevator, 1.25))
            .whileFalse(new InstantCommand(() -> m_elevator.stopElevatorMotor()));
    }

    public void closeAll()
    {
        m_elevator.close();
        Commands.print("Closing all subsystems");
    }

	public Command getAutonomousCommand() 
	{
		return Commands.print("No autonomous command configured");
    }
}
