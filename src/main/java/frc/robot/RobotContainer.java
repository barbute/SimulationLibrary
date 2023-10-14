// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SingledJointedCommand;
import frc.robot.simulations.SingleJointedArm;

public class RobotContainer 
{
    private CommandXboxController m_pilotController = new CommandXboxController(0);

    private SingleJointedArm m_singleJointedArmSim = new SingleJointedArm();

	public RobotContainer() 
	{
		configureBindings();
	}

	private void configureBindings() 
    {
        m_pilotController.a()
            .whileTrue(new SingledJointedCommand(m_singleJointedArmSim, Units.degreesToRadians(200.0)))
            .whileFalse(new InstantCommand(
                () -> m_singleJointedArmSim.stopArmMotor()));

        m_pilotController.x()
            .whileTrue(new SingledJointedCommand(m_singleJointedArmSim, Units.degreesToRadians(90.0)))
            .whileFalse(new InstantCommand(
                () -> m_singleJointedArmSim.stopArmMotor()));

        m_pilotController.b().whileTrue(Commands.runOnce(
            () -> m_singleJointedArmSim.setArmVolts(6.0), 
            m_singleJointedArmSim)
        ).whileFalse(new InstantCommand(
            () -> m_singleJointedArmSim.stopArmMotor()));

        m_pilotController.y().whileTrue(Commands.runOnce(
            () -> m_singleJointedArmSim.setArmVolts(-6.0), 
            m_singleJointedArmSim)
        ).whileFalse(new InstantCommand(
            () -> m_singleJointedArmSim.stopArmMotor()));
    }

    public void closeAll()
    {
        m_singleJointedArmSim.close();
        Commands.print("Closing all subsystems");
    }

	public Command getAutonomousCommand() 
	{
		return Commands.print("No autonomous command configured");
    }
}
