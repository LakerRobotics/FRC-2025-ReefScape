// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
  
import java.io.IOException;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.SetSwerveDrive2023;
import frc.robot.simulation.FieldSim;
//import frc.robot.subsystems.ArmLockSubsystem;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDriveSDS;
import frc.robot.subsystems.Elevator;
import frc.robot.CoralSim;
//import frc.robot.subsystems.SwerveDriveRev;
import frc.robot.utils.GamepadUtils;
import frc.robot.RustConstants.Controls;
import frc.robot.RustConstants.Drivetrain;
import frc.robot.RustConstants.OIConstants;
import frc.robot.Constants.USB; 
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
 
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  // private  SwerveDriveRev m_robotDriveSDS;
  // Initialize Limelight NetworkTable
  // private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  //     @SuppressWarnings("unused")
  private Mechanism2d mechanisms = new Mechanism2d(5, 3);
  private MechanismRoot2d root = mechanisms.getRoot("root", 2.5, 0.25);
  private MechanismLigament2d fromRobot = root
            .append(new MechanismLigament2d("fromRobot", Units.inchesToMeters(5.5), 180, 0,
                    new Color8Bit(Color.kWhite)));
  @SuppressWarnings("unused")
  private MechanismLigament2d elevatorBase = root
            .append(new MechanismLigament2d("elevatorBase", Units.inchesToMeters(36), 90, 2,
                    new Color8Bit(Color.kWhite)));
  private MechanismLigament2d elevatorLigament = root
            .append(new MechanismLigament2d("elevatorStage", Units.inchesToMeters(10), 90, 4,
                    new Color8Bit(Color.kOrange)));
  private MechanismLigament2d armLigament = elevatorLigament
            .append(new MechanismLigament2d("armLigament", Units.inchesToMeters(10), 270, 5,
                    new Color8Bit(Color.kRed)));

  PositionTracker positionTracker = new PositionTracker();

  //@Log
  //Drivetrain drivetrain = new Drivetrain();
  SwerveDriveSDS m_robotDriveSDS = new SwerveDriveSDS();
  //@Log
  Elevator elevator = new Elevator(positionTracker, elevatorLigament);
  //@Log
  Arm arm = new Arm(positionTracker, armLigament, elevator::getCarriageComponentPose);
  //@Log
  Intake intake = new Intake();
  //@Log
  Climber climber = new Climber();

  //@Log
  CoralSim coralSim = new CoralSim(m_robotDriveSDS::getPose, arm::getClawComponentPose);

  //    @Log
  //    LEDs leds = new LEDs();

  //    @Log
  //  HoundBrian houndbrian = new HoundBrian(drivetrain, elevator, arm, climber, leds);

  //   @Log
  private final Supplier<Boolean> initialized = GlobalStates.INITIALIZED::enabled;

//   @SendableLog
  CommandScheduler scheduler = CommandScheduler.getInstance();

//   @Log(groups = "gamePieces")
  public Pose3d getCoralPose() {
      Pose3d relativeCoralPose = arm.getClawComponentPose().plus(new Transform3d(0.143, 0, 0, new Rotation3d()));
      return new Pose3d(m_robotDriveSDS.getPose())
              .plus(new Transform3d(relativeCoralPose.getTranslation(), relativeCoralPose.getRotation()))
              .plus(new Transform3d(0, 0, 0, new Rotation3d(0, Math.PI / 2.0, 0)));
    }
          
  //PositionTracker ArmPositionTracker = new PositionTracker();
  //private final Arm m_arm = new Arm(ArmPositionTracker, null, null);
  //private final Intake m_intake = new Intake();
  //private final Elevator m_launcher = new Elevator(null, null);

//  private final FieldSim m_fieldSim;// = new FieldSim(m_robotDrive);
  static PS4Controller leftJoystick  = new PS4Controller(USB.leftJoystick);
  static PS4Controller rightJoystick = new PS4Controller(USB.rightJoystick);

   // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  private SendableChooser<Command> autoChooser;

  RobotContainer(){
     //NamedCommands.registerCommand("ArmJoystickControl", new ArmJoystickControl(m_arm).withTimeout(3));
     //NamedCommands.registerCommand("IntakeSetPower", new IntakeSetPower(m_intake,1).withTimeout(2));

    // Initialize Logitech camera
    //CameraServer.startAutomaticCapture();  // set the arm subsystem to run the "runAutomatic" function continuously when no other command is running

    //m_arm.setDefaultCommand(new RunCommand(() -> m_arm.runAutomatic(), m_arm));

    // set the intake to stop (0 power) when no other command is running
    //m_intake.setDefaultCommand(new RunCommand(() -> m_intake.setPower(0.0), m_intake));

    // configure the launcher to stop when no other command is running
    //m_launcher.setDefaultCommand(new RunCommand(() -> m_launcher.stopLauncher(), m_launcher));

    //final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
 
    // Create and add SysId commands to Shuffleboard
    var commandsTab = Shuffleboard.getTab("Commands");
    
    // Create SysId commands directly from the subsystem methods
    commandsTab.add("SysId Dynamic Forward",     m_robotDriveSDS.sysIdDynamic(SysIdRoutine.Direction.kForward));     
    commandsTab.add("SysId Dynamic Reverse",     m_robotDriveSDS.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    commandsTab.add("SysId Quasistatic Forward", m_robotDriveSDS.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    commandsTab.add("SysId Quasistatic Reverse", m_robotDriveSDS.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    // Register named commands for PathPlanner
    NamedCommands.registerCommand("PrepareScoreL1", RobotCommands.prepareCoralScoreCommand(ScoreLevel.L1, elevator, arm, coralSim));
    NamedCommands.registerCommand("PrepareScoreL2", RobotCommands.prepareCoralScoreCommand(ScoreLevel.L2, elevator, arm, coralSim));
    NamedCommands.registerCommand("PrepareScoreL3", RobotCommands.prepareCoralScoreCommand(ScoreLevel.L3, elevator, arm, coralSim));
    NamedCommands.registerCommand("PrepareScoreL4", RobotCommands.prepareCoralScoreCommand(ScoreLevel.L4, elevator, arm, coralSim));
    
    NamedCommands.registerCommand("ScoreCoral", RobotCommands.scoreCoralCommand(m_robotDriveSDS, elevator, arm, coralSim));
    NamedCommands.registerCommand("PrepareIntakeCoral", RobotCommands.prepareIntakeCoralCommand(elevator, arm, coralSim));
    NamedCommands.registerCommand("IntakeCoral", RobotCommands.intakeCoralCommand(elevator, arm, coralSim));
    

    // Configure the trigger bindings
    configureBindings();
  // m_fieldSim = new FieldSim(m_robotDriveSDS);
  // m_fieldSim.initSim();

  // Configure default commands
    m_robotDriveSDS.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
        () ->
         m_robotDriveSDS.drive(
             -GamepadUtils.squareInput(leftJoystick.getLeftY(), OIConstants.kDriveDeadband),
              -GamepadUtils.squareInput(leftJoystick.getLeftX(), OIConstants.kDriveDeadband),
              -GamepadUtils.squareInput(leftJoystick.getRightX(), OIConstants.kDriveDeadband),
              true,
              false
          ),
        m_robotDriveSDS
      )
    );

    configureAutos();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    new Trigger( 
      () -> 
           Math.abs(rightJoystick.getLeftY()) > 0.03 )
           .onTrue(new RunCommand(() -> climber.setVoltage(rightJoystick.getLeftY()),climber));  
     new Trigger( 
      () -> 
            Math.abs(rightJoystick.getLeftX()) > 0.03 )
            .onTrue(new RunCommand(() -> elevator.setVoltage(rightJoystick.getLeftX()),elevator)); 

    new Trigger( 
            () -> 
                 Math.abs(rightJoystick.getRightY()) > 0.03 )
                 .onTrue(new RunCommand(() -> intake.setRollerVoltage(rightJoystick.getRightY()),intake));
    new Trigger(
      () -> 
                 Math.abs(rightJoystick.getRightX()) > 0.03 )
                 .onTrue(new RunCommand(() -> arm.setVoltage(rightJoystick.getRightX()),arm)); 
  
  //elevtor preset position buttons borrowed from Rust Hounds
  //TODO make our own elevator preset buttons for our robot
   /*   new JoystickButton(rightJoystick, PS4Controller.Button.kSquare.value)
      .whileTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L2, elevator, arm, coralSim));
     
    new JoystickButton(rightJoystick, PS4Controller.Button.kCircle.value)
      .whileTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L3, elevator, arm, coralSim));
                
    new JoystickButton(rightJoystick, PS4Controller.Button.kTriangle.value)
      .whileTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L4, elevator, arm, coralSim)); */

    // DriveTrainReset 
//    new JoystickButton(leftJoystick, PS4Controller.Button.kTriangle.value) 
//    .onTrue(new RunCommand(()-> m_robotDriveSDS.zeroHeading()));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  
  private void configureAutos() {
    // Initialize the autoChooser
    this.autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }
}