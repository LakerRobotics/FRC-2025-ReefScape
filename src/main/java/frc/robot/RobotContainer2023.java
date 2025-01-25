// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
  
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand; 
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SetSwerveDrive2023;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDriveSDS;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Elevator;
//import frc.robot.subsystems.SwerveDriveRev;
import frc.robot.utils.GamepadUtils;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants2023.USB; 
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
 
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer2023 {
//  private static RobotContainer2023 m_robotContainer = new RobotContainer2023();
 
  // The robot's subsystems and commands are defined here...
 // private  SwerveDriveRev m_robotDriveSDS;
  private  SwerveDriveSDS  m_robotDriveREV;
  // Initialize Limelight NetworkTable
    private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
      private final Arm m_arm = new Arm(null, null, null);
  private final Intake m_intake = new Intake();
  private final Elevator m_elevator = new Elevator(null, null);
  //private final ArmLockSubsystem mArmLockSubsystem = new ArmLockSubsystem();
  private int ROBOT;
  private final int PROD = 1;
  private final int DEV = 0;



//  private final FieldSim m_fieldSim;// = new FieldSim(m_robotDrive);

  static PS4Controller leftJoystick  = new PS4Controller(USB.leftJoystick);
  static PS4Controller rightJoystick = new PS4Controller(USB.rightJoystick);

   // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private SwerveDriveSDS m_robotDriveSDS;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer2023() {
    
       m_robotDriveSDS = new SwerveDriveSDS();
      
     //NamedCommands.registerCommand("ArmJoystickControl", new ArmJoystickControl(m_arm).withTimeout(3));
     //NamedCommands.registerCommand("IntakeSetPower", new IntakeSetPower(m_intake,1).withTimeout(2));

     // Initialize Logitech camera
    CameraServer.startAutomaticCapture();  // set the arm subsystem to run the "runAutomatic" function continuously when no other command is running

   // m_arm.setDefaultCommand(new RunCommand(() -> m_arm.runAutomatic(), m_arm));

    // set the intake to stop (0 power) when no other command is running
   // m_intake.setDefaultCommand(new RunCommand(() -> m_intake.setPower(0.0), m_intake));

    // configure the launcher to stop when no other command is running
   // m_launcher.setDefaultCommand(new RunCommand(() -> m_launcher.stopLauncher(), m_launcher));

   final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");



 
    // Configure the trigger bindings
    configureBindings();
    if(ROBOT == DEV){
 //     m_robotDrive = m_robotDriveSDS;
  // m_fieldSim = new FieldSim(m_robotDriveSDS);
  // m_fieldSim.initSim();
    // Configure default commands
    m_robotDriveSDS.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new SetSwerveDrive2023(
                    m_robotDriveSDS,
                    ()-> leftJoystick.getLeftY(),// getY(),
                    ()-> leftJoystick.getLeftX(), //getX(),
                    ()-> leftJoystick.getRightX(),//getZ(),
                  true));
    }
    else{
      m_robotDriveREV.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
        () ->
          m_robotDriveREV.drive(
              -GamepadUtils.squareInput(
                  leftJoystick.getLeftY(), OIConstants.kDriveDeadband),
              -GamepadUtils.squareInput(
                  leftJoystick.getLeftX(), OIConstants.kDriveDeadband),
              -GamepadUtils.squareInput(
                  leftJoystick.getRightX(), OIConstants.kDriveDeadband),
              true,
            true),
      m_robotDriveREV));
    }

    //SETUP AUTONOMOUS CODE
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

    /* 
    // set up arm preset positions
    new JoystickButton(rightJoystick, PS4Controller.Button.kSquare.value)
        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kScoringPosition)));

    new Trigger(
            () ->
                rightJoystick.getL2Axis()
                    > Constants.OIConstants.kTriggerButtonThreshold)
        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kIntakePosition)));

    new JoystickButton(rightJoystick, PS4Controller.Button.kL1.value)
        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kHomePosition)));
     
    new Trigger( 
      () -> 
           Math.abs(rightJoystick.getLeftY()) > 0.03 )
           .onTrue(new RunCommand(() -> m_arm.runManual(rightJoystick.getLeftY()),m_arm));

    // intake controls (run while button is held down, run retract command once when the button is released)
    new Trigger(
            () ->
                rightJoystick.getR2Axis()
                    > Constants.OIConstants.kTriggerButtonThreshold)
        .whileTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kIntakePosition)))
        .onFalse(m_intake.retract());
     
    new JoystickButton(rightJoystick, PS4Controller.Button.kTriangle.value)
        .whileTrue(new RunCommand(() -> m_intake.setPower(-1.0)));

    new Trigger( 
      () -> 
           Math.abs(rightJoystick.getRightX()) > 0.03 )
           .onTrue(new RunCommand(() -> m_intake.runManual(rightJoystick.getRightX()),m_intake));

    // launcher controls (button to pre-spin the launcher and button to launch)
    new JoystickButton(rightJoystick, PS4Controller.Button.kCircle.value)
        //.whileTrue(new RunCommand(() -> m_launcher.runLauncher(), m_launcher));
        .whileTrue(new LauncherAutoPower(m_launcher,0.9,1));

    new JoystickButton(rightJoystick, PS4Controller.Button.kR1.value)
        .onTrue(m_intake.feedLauncher(m_launcher));
  
    new JoystickButton(rightJoystick, PS4Controller.Button.kCross.value)
        .onTrue(new AmpShoot(m_arm, m_launcher, m_intake, m_robotDriveREV));

    new JoystickButton(rightJoystick, PS4Controller.Button.kPS.value)
        .onTrue(new ArmLockEngage(mArmLockSubsystem)); */

    // DriveTrainReset 
    new JoystickButton(leftJoystick, PS4Controller.Button.kTriangle.value) 
    .onTrue(new RunCommand(()-> m_robotDriveREV.zeroHeading()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
// Load the path you want to follow using its name in the GUI
   // PathPlannerPath path = PathPlannerPath.fromPathFile("seth path 2");

    // Create a path following command using AutoBuilder. This will also trigger event markers.
   // return AutoBuilder.followPath(path);
   //TODO Fix path planner

    
  
  public void periodic() {
 //   m_fieldSim.periodic();
  }

//  public static RobotContainer2023 getInstance() {
//     return m_robotContainer;
//  }

  private void configureAutos() {
    SmartDashboard.putData("auton chooser",m_chooser);
    // Set the Defualt Auton
    /*
    m_chooser.setDefaultOption("Shoot Note", new AutoShootSpeaker(m_arm,m_launcher,m_intake,m_robotDriveREV));
    m_chooser.addOption("Launcher Test", new AutoLauncher(m_launcher));
    m_chooser.addOption("Shoot Note ", new AutoShootSpeaker(m_arm,m_launcher,m_intake,m_robotDriveREV));
    m_chooser.addOption("Right Shoot Note ", new AutoRightShootSpeaker(m_arm,m_launcher,m_intake,m_robotDriveREV));
    m_chooser.addOption("Left Shoot Note ", new AutoLeftShootSpeaker(m_arm,m_launcher,m_intake,m_robotDriveREV));
    */
   // m_chooser.addOption("Shoot Note then follow Path ", new AutoShootSpeakerThenFollowPath(m_arm,m_launcher,m_intake,m_robotDriveREV,"Seth Path"));

    //PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");
    //m_chooser.addOption("Path: Example Path", AutoBuilder.followPath(path));
    //PathPlannerPath path2 = PathPlannerPath.fromPathFile("Seth Path 2");
    //m_chooser.addOption("Path: Seth Path 2", AutoBuilder.followPath(path2));

  }

  


}