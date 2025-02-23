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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants2023.CAN;
import frc.robot.Constants2023.Swerve;
import frc.robot.Constants2023.Swerve.ModulePosition;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.HashMap;
import java.util.Map;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants2023.Swerve.*;
//import edu.wpi.first.units.Voltage;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.MetersPerSecond;


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
double kS = 0.00;
double kV = 0.00;
double kA = 0.00;

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
    kS,  // Static friction constant from SysId
    kV,  // Velocity constant from SysId 
    kA   // Acceleration constant from SysId
);

  private final ProfiledPIDController m_xController = new ProfiledPIDController(
    kP_X,  // Proportional gain from SysId
    0,     // Integral gain (usually left at 0)
    kD_X,  // Derivative gain from SysId
    kThetaControllerConstraints
);
  private ProfiledPIDController m_yController =
          new ProfiledPIDController(kP_Y, 0, kD_Y, kThetaControllerConstraints);
  private ProfiledPIDController m_turnController =
          new ProfiledPIDController(kP_Theta, 0, kD_Theta, kThetaControllerConstraints);

  private double m_simYaw;

  private ChassisSpeeds m_lastCommandedSpeeds = new ChassisSpeeds();

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
    RobotConfig config = null;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
     // throw new RuntimeException("Failed to load robot configuration", e);
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(10.0, 1.0, 0.0), // Translation PID constants
                    new PIDConstants(10.0, 1.0, 0.0) // Rotation PID constants
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
    // Scale inputs to actual speeds
    throttle *= kMaxSpeedMetersPerSecond;
    strafe *= kMaxSpeedMetersPerSecond;
    rotation *= kMaxRotationRadiansPerSecond;

    // Create chassis speeds based on whether we want field or robot relative motion
    ChassisSpeeds chassisSpeeds;
    if (isFieldRelative) {
        // In field relative mode, forward is always toward the opponent's wall
        // regardless of robot orientation
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            throttle,  // Forward/backward speed
            strafe,    // Left/right speed
            rotation,  // Rotation speed
            getHeadingRotation2d()  // Current robot angle
        );
    } else {
        // In robot relative mode, forward is the robot's forward
        chassisSpeeds = new ChassisSpeeds(
            throttle,  // Robot's forward/backward
            strafe,    // Robot's left/right
            rotation   // Robot's rotation
        );
    }

    // Log data to SmartDashboard
    SmartDashboard.putNumber("chassis.vx", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("chassis.vy", chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("chassis.vr", chassisSpeeds.omegaRadiansPerSecond);
    SmartDashboard.putNumber("chassis.heading", getHeadingDegrees());

    // Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);
    
    // Make sure no module tries to go faster than the max speed
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, kMaxSpeedMetersPerSecond);

    // Command each swerve module to its desired state
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
   SmartDashboard.putNumber("Robot X Position", m_odometry.getPoseMeters().getMeasureX().in(Meters));
   SmartDashboard.putNumber("Robot Y Position", m_odometry.getPoseMeters().getMeasureY().in(Meters));
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
public ChassisSpeeds getRobotRelativeSpeeds() {
    // Get actual robot-relative speeds from current module states
    return kSwerveKinematics.toChassisSpeeds(getModuleStates());
}

public Command moveVoltageTimeCommand(double voltage, double time) {
    // Convert voltage to percentage (-1 to 1)
    double speedPercentage = voltage / 12.0;
    
    return Commands.sequence(
        Commands.run(() -> 
            // Use existing drive method: (forward, strafe, rotation, fieldRelative, isOpenLoop)
            drive(speedPercentage, 0, 0, false, true)
        )
        .withTimeout(time)
        .andThen(() -> drive(0, 0, 0, false, true))
    ).withName("drivetrain.moveVoltageTime");
}

public void driveRobotRelative(ChassisSpeeds speeds) {
    drive(
        speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond,
        speeds.omegaRadiansPerSecond,
        false,  // Not field relative
        true   // Closed loop
    );
}

// Create a new SysId routine for characterizing the drive.
private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(
        voltage -> {
            // Directly apply the test voltage to all drive motors
            for (SwerveModuleSDS module : m_swerveModules.values()) {
                module.m_driveMotor.setVoltage(voltage.magnitude());
                // Keep modules pointed forward during test
                module.m_turnMotor.setVoltage(0);
            }
        },
        log -> {
            // Calculate averages from all modules
            double avgVelocity = 0;
            double avgPosition = 0;
            
            for (SwerveModuleSDS module : m_swerveModules.values()) {
                avgVelocity += module.getDriveMetersPerSecond();
                avgPosition += module.getDriveMeters();
            }
            
            avgVelocity /= 4.0;
            avgPosition /= 4.0;
            
            
            edu.wpi.first.units.measure.Voltage avgVolts = Volts.of(avgVelocity);  // Create a Voltage measure using the static factory method
            Distance avgPos = Meters.of(avgPosition);
            LinearVelocity avgVel = MetersPerSecond.of(avgVelocity);  // Create a Velocity measure
            log.motor("swerve-drive")
                .voltage(avgVolts)
                .linearPosition(avgPos)
                .linearVelocity(avgVel);
        },
        this
    )
);

  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

}
