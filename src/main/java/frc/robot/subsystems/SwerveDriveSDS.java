// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.SwerveUtils;
import edu.wpi.first.math.trajectory.Trajectory;
//import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
//import com.pathplanner.lib.util.PIDConstants;
//import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.util.PathPlannerLogging;

public class SwerveDriveSDS extends SubsystemBase {
  // Create MAXSwerveModules
  private final SwerveModuleSDS m_frontLeft =
      new SwerveModuleSDS(
          DriveConstants.kFrontLeftDrivingCanId,
          DriveConstants.kFrontLeftTurningCanId,
          DriveConstants.kFrontLeftChassisAngularOffset);

  private final SwerveModuleSDS m_frontRight =
      new SwerveModuleSDS(
          DriveConstants.kFrontRightDrivingCanId,
          DriveConstants.kFrontRightTurningCanId,
          DriveConstants.kFrontRightChassisAngularOffset);

  private final SwerveModuleSDS m_rearLeft =
      new SwerveModuleSDS(
          DriveConstants.kRearLeftDrivingCanId,
          DriveConstants.kRearLeftTurningCanId,
          DriveConstants.kBackLeftChassisAngularOffset);

  private final SwerveModuleSDS m_rearRight =
      new SwerveModuleSDS(
          DriveConstants.kRearRightDrivingCanId,
          DriveConstants.kRearRightTurningCanId,
          DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
private ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          Rotation2d.fromDegrees(m_gyro.getAngle()),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });
   
/** Creates a new DriveSubsystem. */
  public SwerveDriveSDS() {
//m_frontLeft.m_turningSparkMax.setInverted(false);
//m_frontRight.m_drivingSparkMax.setInverted(true);
//m_rearLeft.m_turningSparkMax.setInverted(true);
//m_rearRight.m_turningSparkMax.setInverted(false);

          // Configure AutoBuilder last
     AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose /*resetPose*/, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(0.5, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(0.5, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
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

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });
  }
public ChassisSpeeds getRobotRelativeSpeeds(){
  //TODO:
  //return new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
 // return new ChassisSpeeds(0,0,0);
  return new ChassisSpeeds(m_currentTranslationMag * Math.cos(m_currentTranslationDir), 
  m_currentTranslationMag * Math.sin(m_currentTranslationDir), 
  m_currentRotation);
}

public void 
driveRobotRelative(ChassisSpeeds speeds) {
  drive(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond,-speeds.omegaRadiansPerSecond,false,false);
}

public void driveFieldRelative(ChassisSpeeds speeds){
    // Convert ChassisSpeeds to field-relative if necessary or use as is if already robot-relative
    var fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, m_gyro.getRotation2d());
    drive(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond, fieldRelativeSpeeds.omegaRadiansPerSecond, true, false);
}


  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param rateLimit Whether to enable rate limiting for smoother control.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

SmartDashboard.putNumber("REV1xspeed", xSpeed);
SmartDashboard.putNumber("REV1yspeed", ySpeed);
SmartDashboard.putNumber("REV1rot", rot);

if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate =
            500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir =
            SwerveUtils.StepTowardsCircular(
                m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag
            > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir =
            SwerveUtils.StepTowardsCircular(
                m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);
    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }
SmartDashboard.putNumber("REV2xSpeedCommanded", xSpeedCommanded);
SmartDashboard.putNumber("REV2ySpeedCommanded", ySpeedCommanded);
SmartDashboard.putNumber("REV2m_currentRotation", m_currentRotation);

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;
SmartDashboard.putNumber("REV3xSpeedDelivered", xSpeedDelivered);
SmartDashboard.putNumber("REV3ySpeedDelivered", ySpeedDelivered);
SmartDashboard.putNumber("REV3rotDelivered", rotDelivered);

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered,
                    Rotation2d.fromDegrees(m_gyro.getAngle()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
        SmartDashboard.putNumber("Rev4[0]AngleRadis",swerveModuleStates[0].angle.getRadians());
        SmartDashboard.putNumber("Rev4[0]Speed"     ,swerveModuleStates[0].speedMetersPerSecond);
        SmartDashboard.putNumber("Rev4[0]AngleActual",m_frontLeft.getState().angle.getRadians());
        SmartDashboard.putNumber("Rev4[0]SpeedActual",m_frontLeft.getState().speedMetersPerSecond);
//        SmartDashboard.putNumber("Rev4[0]AngleMotorPower",m_frontLeft.m_turningSparkMax.get());

        SmartDashboard.putNumber("Rev4[1]AngleRadis",swerveModuleStates[1].angle.getRadians());
        SmartDashboard.putNumber("Rev4[1]Speed"     ,swerveModuleStates[1].speedMetersPerSecond);
        SmartDashboard.putNumber("Rev4[1]AngleActual",m_frontRight.getState().angle.getRadians());
        SmartDashboard.putNumber("Rev4[1]SpeedActual",m_frontRight.getState().speedMetersPerSecond);
//        SmartDashboard.putNumber("Rev4[1]AngleMotorPower",m_frontRight.m_turningSparkMax.get());

        SmartDashboard.putNumber("Rev4[2]AngleRadis",swerveModuleStates[2].angle.getRadians());
        SmartDashboard.putNumber("Rev4[2]Speed"     ,swerveModuleStates[2].speedMetersPerSecond);
        SmartDashboard.putNumber("Rev4[2]AngleActual",m_rearLeft.getState().angle.getRadians());
        SmartDashboard.putNumber("Rev4[2]SpeedActual",m_rearLeft.getState().speedMetersPerSecond);
//         SmartDashboard.putNumber("Rev4[2]AngleMotorPower",m_rearLeft.m_turningSparkMax.get());

        SmartDashboard.putNumber("Rev4[3]AngleRadis",swerveModuleStates[3].angle.getRadians());
        SmartDashboard.putNumber("Rev4[3]Speed"     ,swerveModuleStates[3].speedMetersPerSecond);
        SmartDashboard.putNumber("Rev4[3]AngleActual",m_rearRight.getState().angle.getRadians());
        SmartDashboard.putNumber("Rev4[3]SpeedActual",m_rearRight.getState().speedMetersPerSecond);
//        SmartDashboard.putNumber("Rev4[3]AngleMotorPower",m_rearRight.m_turningSparkMax.get());
      }


  /** Sets the wheels into an X formation to prevent movement. */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void followTrajectory(Trajectory.State desiredState) {
    // Assuming desiredState includes pose and velocity
    Pose2d desiredPose = desiredState.poseMeters;

    // You might need to calculate the robot's velocity, rotation, etc., based on desiredState
    // This is a simplified example. You'll need to adjust it based on your trajectory's data and your robot's capabilities

    // Calculate swerve module states to follow the trajectory
    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        // For simplicity, assuming no rotation - you'll likely need to include rotation calculation here
        new ChassisSpeeds(desiredState.velocityMetersPerSecond, 0, 0)
    );

    // Set the calculated swerve module states to the actual modules
    setModuleStates(swerveModuleStates);
  }
}

