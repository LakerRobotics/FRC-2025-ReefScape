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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkClosedLoopController;

import frc.robot.utils.*;

public class SwerveModuleSDS extends SubsystemBase {
  private final int POS_SLOT = 0;
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

  private double m_simDriveEncoderPosition;
  private double m_simDriveEncoderVelocity;
  private double m_simAngleDifference;
  private double m_simTurnAngleIncrement;
  Pose2d m_pose;

  SimpleMotorFeedforward feedforward =
          new SimpleMotorFeedforward(
                  ksDriveVoltSecondsPerMeter,
                  kaDriveVoltSecondsSquaredPerMeter,
                  kvDriveVoltSecondsSquaredPerMeter);

  private final ProfiledPIDController m_turningPIDController
          = new ProfiledPIDController(0.001, 0, 0,
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
//    m_driveEncoder.setPositionConversionFactor(kDriveRevToMeters);
//    m_driveEncoder.setVelocityConversionFactor(kDriveRpmToMetersPerSecond);

    m_turnEncoder = m_turnMotor.getEncoder();
//    m_turnEncoder.setPositionConversionFactor(kTurnRotationsToDegrees);
//    m_turnEncoder.setVelocityConversionFactor(kTurnRotationsToDegrees / 60);

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
    Angle angle = m_angleEncoder.getAbsolutePosition().getValue().minus(new Angle(m_angleOffset));
    m_turnEncoder.setPosition(angle);
    SmartDashboard.putString("CanCoder Units: ", m_angleEncoder.getAbsolutePosition().getUnits()); 
    SmartDashboard.putNumber("CanCoder value", m_angleEncoder.getAbsolutePosition().getValue()); 
  }

  public double getHeadingDegrees() {
    if(RobotBase.isReal())
      return m_turnEncoder.getPosition();
    else
      return m_currentAngle;
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public double getDriveMeters() {
    if(RobotBase.isReal())
      return m_driveEncoder.getPosition();
    else
      return m_simDriveEncoderPosition;
  }
  public double getDriveMetersPerSecond() {
    if(RobotBase.isReal())
      return m_driveEncoder.getVelocity();
    else
      return m_simDriveEncoderVelocity;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = RevUtils.optimize(desiredState, getHeadingRotation2d());

    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / kMaxSpeedMetersPerSecond;
      SmartDashboard.putNumber("SwerveMo2023 OpenLoop drive motor PercentOutupe",percentOutput);
      m_driveMotor.set(percentOutput);
    } else {
      int DRIVE_PID_SLOT = RobotBase.isReal() ? VEL_SLOT : SIM_SLOT;
      m_driveController.setReference(desiredState.speedMetersPerSecond,CANSparkMax.ControlType.kVelocity);
      SmartDashboard.putNumber("SwerveM23ClosedLoop desired Speed",desiredState.speedMetersPerSecond);
      SmartDashboard.putNumber("SwerveM23ClosedLoop motor speed",m_driveMotor.getEncoder().getVelocity());
    }

    double angle =
            (Math.abs(desiredState.speedMetersPerSecond) <= (kMaxSpeedMetersPerSecond * 0.01))
                    ? m_lastAngle
                    : desiredState.angle.getDegrees(); // Prevent rotating module if speed is less than 1%. Prevents Jittering.
    m_turnController.setReference(angle, CANSparkMax.ControlType.kPosition);
      SmartDashboard.putNumber("SwerveM23. desired angle",angle);
      SmartDashboard.putNumber("SwerveM23. actual angle",m_driveEncoder.getPosition());


    if (RobotBase.isSimulation()) {
      simUpdateDrivePosition(desiredState);
//      simTurnPosition(angle);
      m_currentAngle = angle;

    }
  }
  private void simUpdateDrivePosition(SwerveModuleState state) {
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

  private void updateSmartDashboard() {}

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }
}
