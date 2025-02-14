// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot.subsystems;

//import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix.sensors.Pigeon2; 
//import com.ctre.phoenix.unmanaged.Unmanaged;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.unmanaged.Unmanaged;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.SparkMax;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

//import com.revrobotics.spark.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants2023.CAN;
import frc.robot.Constants2023.Swerve;
import frc.robot.Constants2023.Swerve.ModulePosition;
import java.util.HashMap;
import java.util.Map;

import static frc.robot.Constants2023.Swerve.*;


public class SwerveDriveSDS extends SubsystemBase {



  private final HashMap<ModulePosition, SwerveModuleSDS> m_swerveModules =
                new HashMap<>(
                  Map.of(
                          ModulePosition.FRONT_LEFT,
                          new SwerveModuleSDS(
                                  0,
                                  new SparkMax(CAN.frontLeftTurnMotor, SparkMax.MotorType.kBrushless),
                                  new SparkMax(CAN.frontLeftDriveMotor, SparkMax.MotorType.kBrushless),
                                  new CANcoder(CAN.frontLeftCanCoder),
                                  0),
                          ModulePosition.FRONT_RIGHT,
                          new SwerveModuleSDS(
                                  1,
                                  new SparkMax(CAN.frontRightTurnMotor, SparkMax.MotorType.kBrushless),
                                  new SparkMax(CAN.frontRightDriveMotor, SparkMax.MotorType.kBrushless),
                                  new CANcoder(CAN.frontRightCanCoder),
                                  0),
                          ModulePosition.BACK_LEFT,
                          new SwerveModuleSDS(
                                  2,
                                  new SparkMax(CAN.backLeftTurnMotor, SparkMax.MotorType.kBrushless),
                                  new SparkMax(CAN.backLeftDriveMotor, SparkMax.MotorType.kBrushless),
                                  new CANcoder(CAN.backLeftCanCoder),
                                 0),
                          ModulePosition.BACK_RIGHT,
                          new SwerveModuleSDS(
                                  3,
                                  new SparkMax(CAN.backRightTurnMotor, SparkMax.MotorType.kBrushless),
                                  new SparkMax(CAN.backRightDriveMotor, SparkMax.MotorType.kBrushless),
                                  new CANcoder(CAN.backRightCanCoder),
                                  0)));

//private ADXRS450_Gyro gyro = new ADXRS450_Gyro();
// navX MXP using SPI
AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  private SwerveDriveOdometry m_odometry =
          new SwerveDriveOdometry(
                  Swerve.kSwerveKinematics,
                  getHeadingRotation2d(),
                  getModulePositions(),
                  new Pose2d());

  private ProfiledPIDController m_xController =
          new ProfiledPIDController(kP_X, 0, kD_X, kThetaControllerConstraints);
  private ProfiledPIDController m_yController =
          new ProfiledPIDController(kP_Y, 0, kD_Y, kThetaControllerConstraints);
  private ProfiledPIDController m_turnController =
          new ProfiledPIDController(kP_Theta, 0, kD_Theta, kThetaControllerConstraints);

  private double m_simYaw;

  public SwerveDriveSDS() {
    gyro.reset();
((SwerveModuleSDS) m_swerveModules.get(ModulePosition.FRONT_RIGHT)).m_driveMotor.setInverted(true);
((SwerveModuleSDS) m_swerveModules.get(ModulePosition.BACK_RIGHT)).m_driveMotor.setInverted(true);
((SwerveModuleSDS) m_swerveModules.get(ModulePosition.FRONT_LEFT)).m_driveMotor.setInverted(true);
((SwerveModuleSDS) m_swerveModules.get(ModulePosition.BACK_LEFT)).m_driveMotor.setInverted(true); 


    // All other subsystem initialization
    // ...

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

//((SwerveModule2023) m_swerveModules.get(ModulePosition.FRONT_RIGHT)).m_turnMotor.setInverted(true);
//((SwerveModule2023) m_swerveModules.get(ModulePosition.BACK_RIGHT)).m_turnMotor.setInverted(true);
//((SwerveModule2023) m_swerveModules.get(ModulePosition.FRONT_LEFT)).m_turnMotor.setInverted(true);
//((SwerveModule2023) m_swerveModules.get(ModulePosition.BACK_LEFT)).m_turnMotor.setInverted(true); 
 

  public void drive(
          double throttle,
          double strafe,
          double rotation,
          boolean isFieldRelative,
          boolean isOpenLoop) {
    throttle *= kMaxSpeedMetersPerSecond;
    strafe *= kMaxSpeedMetersPerSecond;
    rotation *= kMaxRotationRadiansPerSecond;



    ChassisSpeeds chassisSpeeds;
//    if (isFieldRelative) {
        ChassisSpeeds chassisSpeedsField = ChassisSpeeds.fromFieldRelativeSpeeds(throttle, strafe, rotation, getHeadingRotation2d());
        SmartDashboard.putNumber("chassis.FieldRelative.vx",chassisSpeedsField.vxMetersPerSecond);
        SmartDashboard.putNumber("chassis.FieldRelative.vy",chassisSpeedsField.vyMetersPerSecond);
        SmartDashboard.putNumber("chassis.FieldRelative.Vr",chassisSpeedsField.omegaRadiansPerSecond);
        SmartDashboard.putNumber("chassis.FieldRelative.Heading",getHeadingDegrees());
        SmartDashboard.putNumber("chassis.FieldRelative.HeadingRot2D",getHeadingRotation2d().getDegrees());
//    } else {
        ChassisSpeeds chassisSpeedsRobot = new ChassisSpeeds(throttle, strafe, rotation);
        SmartDashboard.putNumber("chassis.RobotRelative.vx",chassisSpeedsRobot.vxMetersPerSecond);
        SmartDashboard.putNumber("chassis.RobotRelative.vy",chassisSpeedsRobot.vyMetersPerSecond);
        SmartDashboard.putNumber("chassis.RobotRelative.Vr",chassisSpeedsRobot.omegaRadiansPerSecond);
        SmartDashboard.putNumber("chassis.RobotRelative.Heading",getHeadingDegrees());
//    }
    if (isFieldRelative) {
        chassisSpeeds = chassisSpeedsField;
    }
    else{
        chassisSpeeds = chassisSpeedsRobot;
    }

//    ChassisSpeeds chassisSpeeds =
//            isFieldRelative
//                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
//                    throttle, strafe, rotation, getHeadingRotation2d())
//                    : new ChassisSpeeds(throttle, strafe, rotation);
//Temp    SmartDashboard.putNumber("chassis.vx",chassisSpeeds.vxMetersPerSecond);
//Temp    SmartDashboard.putNumber("chassis.vy",chassisSpeeds.vyMetersPerSecond);
//Temp    SmartDashboard.putNumber("chassis.Vr",chassisSpeeds.omegaRadiansPerSecond);
//Temp    SmartDashboard.putNumber("chassis.Heading",getHeadingDegrees());

    SwerveModuleState[] moduleStates = kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, kMaxSpeedMetersPerSecond);

    for (SwerveModuleSDS module : m_swerveModules.values())
      module.setDesiredState(moduleStates[module.getModuleNumber()], isOpenLoop);
  }

  public void setSwerveModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeedMetersPerSecond);

    for (SwerveModuleSDS module : m_swerveModules.values())
      module.setDesiredState(states[module.getModuleNumber()], isOpenLoop);
  }

  public double getHeadingDegrees() {
    return -Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public Pose2d getPoseMeters() {
    return m_odometry.getPoseMeters();
  }

  public Pose2d getPose() {
        return getPoseMeters();
    }


  public SwerveModuleSDS getSwerveModule(int moduleNumber) {
    return m_swerveModules.get(ModulePosition.values()[moduleNumber]);
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
            m_swerveModules.get(ModulePosition.FRONT_LEFT).getState(),
            m_swerveModules.get(ModulePosition.FRONT_RIGHT).getState(),
            m_swerveModules.get(ModulePosition.BACK_LEFT).getState(),
            m_swerveModules.get(ModulePosition.BACK_RIGHT).getState()
    };
  }
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
            m_swerveModules.get(ModulePosition.FRONT_LEFT).getPosition(),
            m_swerveModules.get(ModulePosition.FRONT_RIGHT).getPosition(),
            m_swerveModules.get(ModulePosition.BACK_LEFT).getPosition(),
            m_swerveModules.get(ModulePosition.BACK_RIGHT).getPosition()
    };
  }

  public void updateOdometry() {
    m_odometry.update(getHeadingRotation2d(), getModulePositions());

    for (SwerveModuleSDS module : m_swerveModules.values()) {
      var modulePositionFromChassis =
              kModuleTranslations[module.getModuleNumber()]
                      .rotateBy(getHeadingRotation2d())
                      .plus(getPoseMeters().getTranslation());
      module.setModulePose(
              new Pose2d(
                      modulePositionFromChassis,
                      module.getHeadingRotation2d().plus(getHeadingRotation2d())));
    }
  }
double test=  0;
  private void updateSmartDashboard() {
        SmartDashboard.putNumber("gyro Angle", gyro.getAngle());
test = test +0.01;  
        SmartDashboard.putNumber("test", test);
        SmartDashboard.putNumber("Get Rotation", getHeadingRotation2d().getRotations());
        SmartDashboard.putNumber("Left Front Module. Drive GetAppliedOutput",m_swerveModules.get( ModulePosition.FRONT_LEFT).m_driveMotor.getAppliedOutput());
        SmartDashboard.putNumber("Left Front Module Drive Output current",m_swerveModules.get( ModulePosition.FRONT_LEFT).m_driveMotor.getOutputCurrent());
        SmartDashboard.putNumber("Left Front Module Drive position",m_swerveModules.get( ModulePosition.FRONT_LEFT).m_driveMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Left Front Module Drive velocity",m_swerveModules.get( ModulePosition.FRONT_LEFT).m_driveMotor.getEncoder().getVelocity());
}

  @Override
  public void periodic() {
    updateOdometry();
    updateSmartDashboard();
  }

  @Override
  public void simulationPeriodic() {
    ChassisSpeeds chassisSpeed = kSwerveKinematics.toChassisSpeeds(getModuleStates());
    m_simYaw += chassisSpeed.omegaRadiansPerSecond * 0.02;

    Unmanaged.feedEnable(20);
    SmartDashboard.putString("need to get a simulated Gyro working for simulated mode to work", "Please fix in SwerveDrive");
    
//    gyro.getSimCollection().setRawHeading(-Units.radiansToDegrees(m_simYaw));s
//    pigeon.getSimCollection().setRawHeading(-Units.radiansToDegrees(m_simYaw));
  }
  public void setVoltage(double voltageForMotors){
}
public void resetPose(Pose2d pose) {
  m_odometry.resetPosition(
      Rotation2d.fromDegrees(gyro.getAngle()),
      new SwerveModulePosition[] {
        m_swerveModules.get(ModulePosition.FRONT_LEFT).getPosition(),
        m_swerveModules.get(ModulePosition.FRONT_RIGHT).getPosition(),
        m_swerveModules.get(ModulePosition.BACK_LEFT).getPosition(),
        m_swerveModules.get(ModulePosition.BACK_RIGHT).getPosition()
      },
      pose);
}
public ChassisSpeeds getRobotRelativeSpeeds(){
  //TODO:
  //return new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
 //return new ChassisSpeeds(0,0,0);
 // return new ChassisSpeeds(m_currentTranslationMag * Math.cos(m_currentTranslationDir), 
  //m_currentTranslationMag * Math.sin(m_currentTranslationDir), 
  //m_currentRotation);
  return new ChassisSpeeds(throttle, strafe, rotation);
}

}
