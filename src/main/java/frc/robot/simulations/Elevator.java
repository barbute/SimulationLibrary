// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Class that represents an Elevator simulation
 * 
 * Note that to use this the objects and their respective values will need to be
 * adjusted for your system
 */
public class Elevator extends SubsystemBase implements AutoCloseable
{
    /* ------------------------ Init elevator constants ------------------------ */
    private final double m_kElevatorGearing = 10.0;
    private final double m_kElevatorDrumRadiusM = Units.inchesToMeters(2.0);
    private final double m_kElevatorCarriageMassKg = 4.0;
    private final double m_kElevatorMinHeightM = 0.0;
    private final double m_kElevatorMaxHeightM = 1.25;
    private final double m_kElevatorEncoderDistancePerPulsePPR = 2.0 * Math.PI * m_kElevatorDrumRadiusM / 4096.0;
    private final double m_kElevatorSimulationStepMS = 0.020;

    /* ------------------------ Init elevator objects ------------------------ */
    private double m_elevatorP = 5.0;
    private double m_elevatorI = 0.0;
    private double m_elevatorD = 0.0;

    private TrapezoidProfile.Constraints m_profileConstraints = new TrapezoidProfile.Constraints(
        2.45, 2.45
    );

    private double m_elevatorS = 0.0;
    private double m_elevatorG = 0.762;
    private double m_elevatorV = 0.762;
    private double m_elevatorA = 0.0;

    private ProfiledPIDController m_controller = new ProfiledPIDController(
        m_elevatorP, m_elevatorI, m_elevatorD, m_profileConstraints);
    private ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
        m_elevatorS, m_elevatorG, m_elevatorV, m_elevatorA);

    private Encoder m_elevatorEncoder = new Encoder(0, 1);
    private PWMSparkMax m_elevatorMotor = new PWMSparkMax(0);

    /* ------------------------ Init sim objects ------------------------ */
    private final DCMotor m_kGearbox = DCMotor.getVex775Pro(4);

    private final ElevatorSim m_kElevatorSim = new ElevatorSim(
        m_kGearbox, 
        m_kElevatorGearing, 
        m_kElevatorCarriageMassKg, 
        m_kElevatorDrumRadiusM, 
        m_kElevatorMinHeightM, 
        m_kElevatorMaxHeightM, 
        true,
        VecBuilder.fill(0.01)
    );

    private final EncoderSim m_kElevatorEncoderSim = new EncoderSim(m_elevatorEncoder);
    private final PWMSim m_kElevatorMotorSim = new PWMSim(m_elevatorMotor);

    /* ------------------------ Init mechanism constants ------------------------ */
    private final double m_kMechDisplayWidth = 2.0;
    private final double m_kMechDisplayHeight = 5.0;

    private final String m_kMechElevatorRootKey = "ElevatorRoot";
    private final double m_kMechElevatorRootX = 1.0;
    private final double m_kMechElevatorRootY = 0.0;

    private final String m_kMechElevatorKey = "Elevator";
    private final double m_kMechElevatorAngle = 90.0;

    /* ------------------------ Init mechanism visualisers ------------------------ */
    /** Display window for the simulated mechanism (like a mechanism's space) */
    private final Mechanism2d m_kMechanismDisplay = new Mechanism2d(m_kMechDisplayWidth, m_kMechDisplayHeight);

    /** Base position for the elevator */
    private final MechanismRoot2d m_kMechElevatorRoot = m_kMechanismDisplay.getRoot(
        // Root is at X and Y
        m_kMechElevatorRootKey, m_kMechElevatorRootX, m_kMechElevatorRootY
    );

    /** Elevator carriage */
    private final MechanismLigament2d m_kMechElevator = m_kMechElevatorRoot.append(
        // Elevator is at a 90 degree angle originating at m_kMechElevatorRoot
        new MechanismLigament2d(m_kMechElevatorKey, m_kElevatorSim.getPositionMeters(), m_kMechElevatorAngle)
    );

    /** Constructor, configures objects */
    public Elevator() 
    {
        // Configure encoder
        m_elevatorEncoder.setDistancePerPulse(m_kElevatorEncoderDistancePerPulsePPR);

        // Put mechanism to network table
        SmartDashboard.putData("ElevatorSimDisplay", m_kMechanismDisplay);
    }
    
    @Override
    public void periodic()
    {
        m_kMechElevator.setLength(m_elevatorEncoder.getDistance());
    }

    @Override
    public void simulationPeriodic() 
    {
        // Set the simulator's input voltages
        m_kElevatorSim.setInput(m_kElevatorMotorSim.getSpeed() * RobotController.getBatteryVoltage());

        // Step the simulator
        m_kElevatorSim.update(m_kElevatorSimulationStepMS);

        // Update simulated encoder
        m_kElevatorEncoderSim.setDistance(m_kElevatorSim.getPositionMeters());
        // Update simulated battery
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(m_kElevatorSim.getCurrentDrawAmps())
        );
    }

    /**
     * Moves the elevator to a setpoint
     * 
     * @param setpoint Goal to reach
     */
    public void elevatorToSetpoint(double setpoint)
    {
        m_controller.setGoal(setpoint);

        var controllerOutput = m_controller.calculate(m_elevatorEncoder.getDistance());
        var feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);

        setElevatorVolts(controllerOutput + feedforwardOutput);
    }

    /**
     * Sets the voltage for the elevator motor. Will clamp -12 to 12
     * 
     * @param volts Volts
     */
    public void setElevatorVolts(double volts)
    {
        final var kClampedVolts = MathUtil.clamp(volts, -12.0, 12.0);

        m_elevatorMotor.setVoltage(kClampedVolts);
    }

    /** Set motor speed to 0 */
    public void stopElevatorMotor()
    {
        m_elevatorMotor.stopMotor();
    }

    @Override
    public void close() 
    {
        m_elevatorMotor.close();
        m_elevatorEncoder.close();

        m_kMechanismDisplay.close();
        m_kMechElevatorRoot.close();
    }
}
