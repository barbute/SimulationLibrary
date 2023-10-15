// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulations;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
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
public class DoubleJointedArm extends SubsystemBase implements AutoCloseable
{
    /* ------------------------ Init arm constants ------------------------ */
    private final double m_kWireMassKg = Units.lbsToKilograms(2.84);

    private final double m_kJoint1MassKg = Units.lbsToKilograms(3.54) + (m_kWireMassKg / 2.0);
    private final double m_kJoint1LengthM = Units.inchesToMeters(25.0);
    private final double m_kJoint1CgRadiusM = Units.inchesToMeters(12.5);
    private final double m_kJoint1MoI = SingleJointedArmSim.estimateMOI(m_kJoint1LengthM, m_kJoint1MassKg);
    private final double m_kJoint1Gearing = 100.0;

    private final double m_kJoint2MassKg = Units.lbsToKilograms(3.08) + (m_kWireMassKg / 2.0);
    private final double m_kJoint2LengthM = Units.inchesToMeters(30.0);
    private final double m_kJoint2CgRadiusM = Units.inchesToMeters(15.0);
    private final double m_kJoint2MoI = SingleJointedArmSim.estimateMOI(m_kJoint2LengthM, m_kJoint2MassKg);
    private final double m_kJoint2Gearing = 100.0;

    private final double m_kJoint1P = 0.1100;
    private final double m_kJoint1I = 0.02;
    private final double m_kJoint1D = 0.0003;

    private final double m_kJoint1S = 0.0;
    private final double m_kJoint1G = 1.1;
    private final double m_kJoint1V = 0.0;
    private final double m_kJoint1A = 0.0;

    private final TrapezoidProfile.Constraints m_kJoint1Constraints = new TrapezoidProfile.Constraints(
        600.0, 140.0
    );

    private final double m_kJoint2P = 0.03;
    private final double m_kJoint2I = 0.2;
    private final double m_kJoint2D = 0.0056;

    private final double m_kJoint2S = 0.0;
    private final double m_kJoint2G = 0.175;
    private final double m_kJoint2V = 0.0;
    private final double m_kJoint2A = 0.0;

    private final TrapezoidProfile.Constraints m_kJoint2Constraints = new TrapezoidProfile.Constraints(
        600.0, 160.0
    );

    /* ------------------------ Init arm objects ------------------------ */
    // How a double-jointed arm moves
    private DoubleJointedArmDynamics m_armDynamics = new DoubleJointedArmDynamics(
        new DoubleJointedArmDynamics.JointConfig(
            m_kJoint1MassKg, 
            m_kJoint1CgRadiusM, 
            m_kJoint1LengthM, 
            m_kJoint1MoI, 
            m_kJoint1Gearing, 
            DCMotor.getNEO(1)
        ), 
        new DoubleJointedArmDynamics.JointConfig(
            m_kJoint2MassKg, 
            m_kJoint2CgRadiusM, 
            m_kJoint2LengthM, 
            m_kJoint2MoI, 
            m_kJoint2Gearing, 
            DCMotor.getNEO(1)
        ) 
    );

    private ProfiledPIDController m_joint1Controller = new ProfiledPIDController(
        m_kJoint1P, m_kJoint1I, m_kJoint1D, m_kJoint1Constraints
    );

    private ProfiledPIDController m_joint2Controller = new ProfiledPIDController(
        m_kJoint2P, m_kJoint2I, m_kJoint2D, m_kJoint2Constraints
    );

    private ArmFeedforward m_joint1Feedforward = new ArmFeedforward(m_kJoint1S, m_kJoint1G, m_kJoint1V, m_kJoint1A);
    private ArmFeedforward m_joint2Feedforward = new ArmFeedforward(m_kJoint2S, m_kJoint2G, m_kJoint2V, m_kJoint2A);

    // Init Vectors:
    // Position = N1 - N2
    // Velocity = N3 - N4
    private Vector<N4> m_armState = VecBuilder.fill(Math.toRadians(0.0), Math.toRadians(0.0), 0.0, 0.0);

    private PWMSparkMax m_joint1Motor = new PWMSparkMax(0);
    private PWMSparkMax m_joint2Motor = new PWMSparkMax(1);

    /* ------------------------ Init sim objects ------------------------ */
    private final double m_kArmSimulationStepMS = 0.020;

    /* ------------------------ Init mechanism constants ------------------------ */
    private final double m_kMechDisplayWidth = 60.0;
    private final double m_kMechDisplayHeight = 60.0;

    private final String m_kMechArmPivotKey = "ArmPivot";
    private final double m_kMechArmPivotX = 30.0;
    private final double m_kMechArmPivotY = 25.0;

    private final String m_kMechArmTowerKey = "ArmTower";
    private final double m_kMechArmTowerLengthM = 50.0 / 2.0;
    private final double m_kMechArmTowerAngleDeg = -90.0;
    private final Color8Bit m_kMechArmTowerColor = new Color8Bit(Color.kWhite);

    private final String m_kMechArmJoint1Key = "ArmJoint1";
    private final double m_kMechArmJoint1LengthIn = Units.metersToInches(m_kJoint1LengthM) / 2.0;
    private final double m_kMechArmJoint1AngleDeg = Units.radiansToDegrees(m_armState.get(0, 0));
    private final Color8Bit m_kMechArmJoint1Color = new Color8Bit(Color.kAqua);

    private final String m_kMechArmJoint2Key = "ArmJoint2";
    private final double m_kMechArmJoint2LengthIn = Units.metersToInches(m_kJoint2LengthM) / 2.0;
    private final double m_kMechArmJoint2AngleDeg = Units.radiansToDegrees(m_armState.get(2, 0));
    private final Color8Bit m_kMechArmJoint2Color = new Color8Bit(Color.kBlue);

    /* ------------------------ Init mechanism visualisers ------------------------ */
    private final Mechanism2d m_kMechanismDisplay = new Mechanism2d(m_kMechDisplayWidth, m_kMechDisplayHeight);

    private final MechanismRoot2d m_kMechArmPivot = m_kMechanismDisplay.getRoot(
        m_kMechArmPivotKey, m_kMechArmPivotX, m_kMechArmPivotY
    );
    private final MechanismLigament2d m_kMechArmTower = m_kMechArmPivot.append(
        new MechanismLigament2d(m_kMechArmTowerKey, m_kMechArmTowerLengthM, m_kMechArmTowerAngleDeg)
    );

    private final MechanismLigament2d m_kMechArmJoint1 = m_kMechArmPivot.append(
        // Append joint 1 to top of tower (Pivot is at top of tower)
        new MechanismLigament2d(
            m_kMechArmJoint1Key, 
            m_kMechArmJoint1LengthIn, 
            m_kMechArmJoint1AngleDeg, 
            10.0, 
            m_kMechArmJoint1Color
        )
    );
    private final MechanismLigament2d m_kMechArmJoint2 = m_kMechArmJoint1.append(
        // Append joint 2 to joint 1
        new MechanismLigament2d(
            m_kMechArmJoint2Key, 
            m_kMechArmJoint2LengthIn, 
            m_kMechArmJoint2AngleDeg, 
            10.0, 
            m_kMechArmJoint2Color
        )
    );

    /** Constructor, configures objects */
    public DoubleJointedArm() 
    {
        // Configure controllers
        m_joint1Controller.enableContinuousInput(-180.0, 180.0);
        m_joint2Controller.enableContinuousInput(-180.0, 180.0);
        
        m_joint1Controller.setIntegratorRange(-0.08, 0.09);
        m_joint2Controller.setIntegratorRange(-0.9, 0.7);

        m_joint1Controller.reset(m_armState.get(0, 0));
        m_joint2Controller.reset(m_armState.get(1, 0));

        // Put mechanism to network table
        m_kMechArmTower.setColor(m_kMechArmTowerColor);
        SmartDashboard.putData("ArmSimDisplay", m_kMechanismDisplay);
    }

    @Override
    public void simulationPeriodic() 
    {
        m_armState = m_armDynamics.simulate(
            m_armState, 
            VecBuilder.fill(
                m_joint1Motor.get() * 12.0,
                m_joint2Motor.get() * 12.0
            ), 
            m_kArmSimulationStepMS
        );

        m_kMechArmJoint1.setAngle(Units.radiansToDegrees(m_armState.get(0, 0)));
        // Subtracts ligament 1's angle since it is relative to ligament 1
        m_kMechArmJoint2.setAngle(
            Units.radiansToDegrees(m_armState.get(1, 0) - m_armState.get(0, 0))
        );
    }
    
    /** Moves the arm to a pre defined setpoint */
    public void armToSetpoint()
    {
        var controller1Output = m_joint1Controller.calculate(
            Math.toDegrees(m_armState.get(0, 0)), 135.0
        );
        var controller2Output = m_joint1Controller.calculate(
            Math.toDegrees(m_armState.get(1, 0)), 30.0
        );

        var feedforward1Output = m_joint1Feedforward.calculate(
            Math.toRadians(m_joint1Controller.getSetpoint().position), 
            Math.toRadians(m_joint1Controller.getSetpoint().velocity)
        );
        var feedforward2Output = m_joint2Feedforward.calculate(
            Math.toRadians(m_joint2Controller.getSetpoint().position), 
            Math.toRadians(m_joint2Controller.getSetpoint().velocity)
        );

        m_joint1Motor.set(controller1Output + feedforward1Output);
        m_joint2Motor.set(controller2Output + feedforward2Output);
    }

    /** Stops the arm motors */
    public void stopArmMotors()
    {
        m_joint1Motor.stopMotor();
        m_joint2Motor.stopMotor();
    }

    @Override
    public void close() 
    {
        m_joint1Motor.close();
        m_joint2Motor.close();

        m_kMechanismDisplay.close();
        m_kMechArmPivot.close();
        m_kMechArmJoint1.close();
        m_kMechArmJoint2.close();
    }
}
