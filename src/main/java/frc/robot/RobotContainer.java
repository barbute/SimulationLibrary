// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DoubleJointedCommand;
import frc.robot.simulations.DoubleJointedArm;

public class RobotContainer 
{
    private CommandXboxController m_pilotController = new CommandXboxController(0);

    private DoubleJointedArm m_doubleJointedArmSim = new DoubleJointedArm();

	public RobotContainer() 
	{
		configureBindings();
	}

	private void configureBindings() 
    {
        m_pilotController.a()
            .whileTrue(new DoubleJointedCommand(m_doubleJointedArmSim))
            .whileFalse(new InstantCommand(
                () -> m_doubleJointedArmSim.stopArmMotors())
        );
    }

    public void closeAll()
    {
        m_doubleJointedArmSim.close();
        Commands.print("Closing all subsystems");
    }

	public Command getAutonomousCommand() 
	{
		return Commands.print("No autonomous command configured");
    }
}
