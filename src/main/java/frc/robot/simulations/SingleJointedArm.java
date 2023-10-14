// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Class that represents a Single-Jointed Arm simulation
 * 
 * Note that to use this the objects and their respective values will need to be
 * adjusted for your system
 */
public class SingleJointedArm extends SubsystemBase implements AutoCloseable
{
    /* ------------------------ Init arm constants ------------------------ */
    private final double m_kArmReduction = 200.0;
    private final double m_kArmLengthM = Units.inchesToMeters(27.0);
    private final double m_kArmMassKg = 8.0;
    private final double m_kArmMoI = SingleJointedArmSim.estimateMOI(m_kArmLengthM, m_kArmMassKg);
    private final double m_kArmMinAngleRads = Units.degreesToRadians(-75.0);
    private final double m_kArmMaxAngleRads = Units.degreesToRadians(255.0);
    private final double m_kArmEncoderDistancePerPulseRads = 2.0 * Math.PI / 4096.0; // Radians
    private final double m_kArmSimulationStepMS = 0.020;

    /* ------------------------ Init arm objects ------------------------ */
    private double m_armP = 10.0;
    private double m_armI = 0.0;
    private double m_armD = 0.0;

    private PIDController m_armController = new PIDController(m_armP, m_armI, m_armD);
    private Encoder m_armEncoder = new Encoder(0, 1);
    private final PWMSparkMax m_kArmMotor = new PWMSparkMax(0);

    private DCMotor m_armGearbox = DCMotor.getVex775Pro(1);

    /* ------------------------ Init sim objects ------------------------ */
    private final SingleJointedArmSim m_kArmSim = new SingleJointedArmSim(
        m_armGearbox, 
        m_kArmReduction, 
        m_kArmMoI, 
        m_kArmLengthM, 
        m_kArmMinAngleRads, 
        m_kArmMaxAngleRads, 
        true,
        VecBuilder.fill(m_kArmEncoderDistancePerPulseRads)
    );

    private final EncoderSim m_kArmEncoderSim = new EncoderSim(m_armEncoder);

    /* ------------------------ Init mechanism constants ------------------------ */
    private final double m_kMechDisplayWidth = 60.0;
    private final double m_kMechDisplayHeight = 60.0;

    private final String m_kMechArmPivotKey = "ArmPivot";
    private final double m_kMechArmPivotX = 30.0;
    private final double m_kMechArmPivotY = 30.0;

    private final String m_kMechArmTowerKey = "ArmTower";
    private final double m_kMechArmTowerLengthIn = 30.0;
    private final double m_kMechArmTowerAngleDeg = -90.0;
    private final Color8Bit m_kMechArmTowerColor = new Color8Bit(Color.kAqua);

    private final String m_kMechArmBicepKey = "ArmBicep";
    private final double m_kMechArmBicepLengthIn = 30.0;
    private final double m_kMechArmBicepWidth = 6.0;
    private final Color8Bit m_kMechArmBicepColor = new Color8Bit(Color.kWhite);

    /* ------------------------ Init mechanism visualisers ------------------------ */
    /** Display window for the simulated mechanism (like a mechanism's space) */
    private final Mechanism2d m_kMechanismDisplay = new Mechanism2d(m_kMechDisplayWidth, m_kMechDisplayHeight);

    /** Pivot of the arm (axis that it rotates around) */
    private final MechanismRoot2d m_kMechArmPivot = m_kMechanismDisplay.getRoot(
        m_kMechArmPivotKey, 
        m_kMechArmPivotX, 
        m_kMechArmPivotY
    );

    /** Tower that the pivot is mounted on */
    private final MechanismLigament2d m_kMechArmTower = m_kMechArmPivot.append(
        // Tower ligament, XY is where it ends from the pivots
        new MechanismLigament2d(
            m_kMechArmTowerKey, 
            m_kMechArmTowerLengthIn, 
            m_kMechArmTowerAngleDeg
        )
    );

    /** The actual arm ligament mounted on the pivot */
    private final MechanismLigament2d m_kMechArmBicep = m_kMechArmPivot.append(
        // Ligament, param 3 is the intial angle of the arm
        new MechanismLigament2d(
            m_kMechArmBicepKey, 
            m_kMechArmBicepLengthIn, 
            Units.radiansToDegrees(m_kArmSim.getAngleRads()),
            m_kMechArmBicepWidth,
            m_kMechArmBicepColor
        )
    );

	/** Constructor, configures objects */
	public SingleJointedArm() 
    {
        // Configure encoder
        m_armEncoder.setDistancePerPulse(m_kArmEncoderDistancePerPulseRads);

        // Configure controller
        m_armController.enableContinuousInput(-180.0, 180.0);

        // Put mechanism to network table
        SmartDashboard.putData("ArmSimDisplay",m_kMechanismDisplay);
        m_kMechArmTower.setColor(m_kMechArmTowerColor);
    }

	@Override
	public void simulationPeriodic() 
	{
        // Set simulated input voltages
		m_kArmSim.setInput(m_kArmMotor.get() * RobotController.getBatteryVoltage());

        // Update simulation (step)
        m_kArmSim.update(m_kArmSimulationStepMS);

        // Set encoder distance based on the simulation
        m_kArmEncoderSim.setDistance(m_kArmSim.getAngleRads());

        // Simulate battery voltages
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(m_kArmSim.getCurrentDrawAmps())
        );

        // Set the visualizer's angle based on simulation
        m_kMechArmBicep.setAngle(Units.radiansToDegrees(m_kArmSim.getAngleRads()));
	}

    /**
     * Move the arm to a setpoint
     * 
     * @param setpointRads Positional goal in rads
     */
    public void armToSetpoint(double setpointRads)
    {
        var controllerOutput = m_armController.calculate(m_armEncoder.getDistance(), setpointRads);

        setArmVolts(controllerOutput);
    }

    /**
     * Sets the voltage for the arm motor. Will clamp -12 to 12
     * 
     * @param volts Volts
     */
    public void setArmVolts(double volts)
    {
        final var kClampedVolts = MathUtil.clamp(volts, -12.0, 12.0);

        m_kArmMotor.setVoltage(kClampedVolts);
    }

    /** Set motor speed to 0 */
    public void stopArmMotor()
    {
        m_kArmMotor.stopMotor();
    }

    @Override
    public void close() 
    {
        m_kArmMotor.close();
        m_armEncoder.close();
        m_armController.close();

        m_kMechanismDisplay.close();
        m_kMechArmPivot.close();
        m_kMechArmBicep.close();
    }
}
