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

    /* ------------------------ Init mechanism constants ------------------------ */

    /* ------------------------ Init mechanism visualisers ------------------------ */

    /*
   
    // ARM VISUALIZER
    private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivotDJ", 30, 25);
    private final MechanismLigament2d m_armTower =
        m_armPivot.append(new MechanismLigament2d("ArmTowerDJ", 50/2, -90));
    private final MechanismLigament2d m_arm =
        m_armPivot.append(
            new MechanismLigament2d(
              "ArmDJ",
              Units.metersToInches(joint1Length)/2,
              Units.radiansToDegrees(armState.get(0, 0)),
              10,
              yellow));
    private final MechanismLigament2d m_arm2 = 
        m_arm.append(
            new MechanismLigament2d(
              "Arm2DJ",
              Units.metersToInches(joint2Length)/2,
              Units.radiansToDegrees(armState.get(1, 0)),
              10,
              green
              ));
     */

    /** Constructor, configures objects */
    public DoubleJointedArm() {}

    @Override
    public void simulationPeriodic() {}

    @Override
    public void close() {}
}
