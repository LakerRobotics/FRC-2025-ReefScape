// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import static frc.robot.Constants2023.Swerve.Module.*;
import static frc.robot.Constants2023.Swerve.kMaxSpeedMetersPerSecond;
//import static frc.robot.Constants.SwerveModuleConstants.*;

 
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.*;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import edu.wpi.first.math.util.Units;

//import edu.wpi.first.units.AngleUnit;
//import edu.wpi.first.units.Units;  // Remove edu.wpi.first.math.util.Units
//import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;

//import frc.robot.utils.*;

public class SwerveModuleSDS extends SubsystemBase {
//  private final int POS_SLOT = 0;
  private final int VEL_SLOT = 1;
  private final int SIM_SLOT = 2;

  int m_moduleNumber;
  SparkMax m_turnMotor;
  SparkMax m_driveMotor;
  private final SparkClosedLoopController m_driveController;
  private SparkClosedLoopController m_turnController;
  public final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turnEncoder;
  CANcoder m_angleEncoder;
  double m_angleOffset;
  double m_currentAngle;
  double m_lastAngle;

//  private double m_simDriveEncoderPosition;
//  private double m_simDriveEncoderVelocity;
//  private double m_simAngleDifference;
//  private double m_simTurnAngleIncrement;
  Pose2d m_pose;

  SimpleMotorFeedforward feedforward =
          new SimpleMotorFeedforward(
                  ksDriveVoltSecondsPerMeter,
                  kvDriveVoltSecondsSquaredPerMeter,
                  kaDriveVoltSecondsSquaredPerMeter);

  private final ProfiledPIDController m_turningPIDController
          = new ProfiledPIDController(1.0, 0.0, 0.1,  // Increased P gain and added D gain
          new TrapezoidProfile.Constraints(2 * Math.PI, 2 * Math.PI));
          //new TrapezoidProfile.Constraints(2 * Math.PI, 2 * Math.PI));
 
  public SwerveModuleSDS(
          int moduleNumber,
          SparkMax turnMotor,
          SparkMax driveMotor,
          CANcoder angleEncoder,
          double angleOffset) {
//    turnMotor.setSecondaryCurrentLimit(1);
//    driveMotor.setSecondaryCurrentLimit(1);
    m_moduleNumber = moduleNumber;
    m_turnMotor = turnMotor;
    m_driveMotor = driveMotor;
    m_angleEncoder = angleEncoder;
    m_angleOffset = angleOffset;

    //m_driveMotor.restoreFactoryDefaults();
    //RevUtils.setDriveMotorConfig(m_driveMotor);
    //m_driveMotor.  setIdleMode();   (SparkMax.IdleMode.kBrake);

    //m_turnMotor.restoreFactoryDefaults();
    //RevUtils.setTurnMotorConfig(m_turnMotor);
    //m_turnMotor.setIdleMode(SparkMax.IdleMode.kBrake);

//    m_angleEncoder.configFactoryDefault();
//    m_angleEncoder.configAllSettings(CtreUtils.generateCanCoderConfig());
//TODO RGT fix    
//m_angleEncoder.configAllSettings(convertPulseToDegree);

    m_driveEncoder = m_driveMotor.getEncoder();
    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig.encoder
        .positionConversionFactor(kDriveRevToMeters)
        .velocityConversionFactor(kDriveRpmToMetersPerSecond);
    
    m_driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_turnEncoder = m_turnMotor.getEncoder();
    SparkMaxConfig turnConfig = new SparkMaxConfig();
    turnConfig.inverted(true);
    turnConfig.encoder
        .positionConversionFactor(kTurnRotationsToDegrees)
        .velocityConversionFactor(kTurnRotationsToDegrees / 60);
    // Add PID values for turning
    turnConfig.closedLoop
        .p(0.02)   // Using last year's proven value;
        .i(0.0)
        .d(0.0);
//        .ff(0.0);
    
    m_turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_driveController = m_driveMotor.getClosedLoopController();// getPIDController();
    m_turnController = m_turnMotor.getClosedLoopController(); //getPIDController();

    if (RobotBase.isSimulation()) {
//      REVPhysicsSim.getInstance().addSparkMax(m_driveMotor, DCMotor.getNEO(1));
//      REVPhysicsSim.getInstance().addSparkMax(m_turnMotor, DCMotor.getNEO(1));
//      m_driveController.setP(1, SIM_SLOT);
    }

    resetAngleToAbsolute();
  }

  public int getModuleNumber() {
    return m_moduleNumber;
  }

  public void resetAngleToAbsolute() {
    // Convert CANcoder absolute position to module angle
    Angle absolutePosition = m_angleEncoder.getAbsolutePosition().getValue();
    Angle offsetPosition = absolutePosition.minus(Units.Degrees.of(m_angleOffset));
    double positionDegrees = offsetPosition.in(Units.Degrees);
    
    m_turnEncoder.setPosition(positionDegrees);
    
    // Debug output
    SmartDashboard.putNumber("Module " + m_moduleNumber + " Absolute Angle", 
        absolutePosition.in(Units.Degrees));
    SmartDashboard.putNumber("Module " + m_moduleNumber + " Offset Angle", 
        positionDegrees);
  }

  public double getHeadingDegrees() {
    return m_turnEncoder.getPosition(); // The position will already be in degrees due to conversion factor
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public double getDriveMeters() {
    return m_driveEncoder.getPosition(); // The position will already be in meters due to conversion factor
  }

  public double getDriveMetersPerSecond() {
    return m_driveEncoder.getVelocity(); // The velocity will already be in meters per second due to conversion factor
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Optimize the desired state to avoid spinning more than 90 degrees
    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getHeadingRotation2d());

    if (isOpenLoop) {
      double percentOutput = optimizedState.speedMetersPerSecond / kMaxSpeedMetersPerSecond;
      m_driveMotor.set(percentOutput);
      SmartDashboard.putNumber("Module OpenLoop" + m_moduleNumber + " Drive %", percentOutput);
    } else {
      int DRIVE_PID_SLOT = RobotBase.isReal() ? VEL_SLOT : SIM_SLOT;
      double velocity = optimizedState.speedMetersPerSecond;
      
      // Use both feedback and feedforward for better control
      double feedforwardVolts = feedforward.calculate(velocity);
//      m_driveMotor.setVoltage(feedforwardVolts);
//      SmartDashboard.putNumber("Module NotOpenLoop" + m_moduleNumber + " feedForwardVolts %", feedforwardVolts);
      // The new API requires using a ClosedLoopSlot object
      ClosedLoopSlot slot = ClosedLoopSlot.kSlot0;
      m_driveController.setReference(
        velocity, 
        SparkMax.ControlType.kVelocity,
        slot,
        feedforwardVolts  // Add feedforward voltage
      );
    }

    double angle =
            (Math.abs(optimizedState.speedMetersPerSecond) <= (kMaxSpeedMetersPerSecond * 0.01))
                    ? m_lastAngle
                    : optimizedState.angle.getDegrees(); // Prevent rotating module if speed is less than 1%. Prevents Jittering.
    m_turnController.setReference(angle, SparkMax.ControlType.kPosition);
      SmartDashboard.putNumber("Module" + m_moduleNumber + " desired angle",angle);
      SmartDashboard.putNumber("Module" + m_moduleNumber + " actual angle",m_driveEncoder.getPosition());

/* 
    if (RobotBase.isSimulation()) {
      simUpdateDrivePosition(optimizedState);
//      simTurnPosition(angle);
      m_currentAngle = angle;

    }
    */
  }
  /*private void simUpdateDrivePosition(SwerveModuleState state) {
    m_simDriveEncoderVelocity = state.speedMetersPerSecond;
    double distancePer20Ms = m_simDriveEncoderVelocity / 50.0;

    m_simDriveEncoderPosition += distancePer20Ms;
  }
  private void simTurnPosition(double angle) {
    if (angle != m_currentAngle && m_simTurnAngleIncrement == 0) {
      m_simAngleDifference = angle - m_currentAngle;
      m_simTurnAngleIncrement = m_simAngleDifference / 20.0;// 10*20ms = .2 sec move time
    }

    if (m_simTurnAngleIncrement != 0) {
      m_currentAngle += m_simTurnAngleIncrement;

      if ((Math.abs(angle - m_currentAngle)) < .1) {
        m_currentAngle = angle;
        m_simTurnAngleIncrement = 0;
      }
    }
  }
*/
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveMetersPerSecond(), getHeadingRotation2d());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDriveMeters(), getHeadingRotation2d());
  }
  public void setModulePose(Pose2d pose) {
    m_pose = pose;
  }

  public Pose2d getModulePose() {
    return m_pose;
  }


  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    //REVPhysicsSim.getInstance().run();
  }
}
