package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj.livewindow.LiveWindow;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import org.usfirst.frc0.SwerveModule.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
 
import com.revrobotics.spark.SparkMax;
  
/** 
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in 
 * the project.
 */
public class Robot extends TimedRobot {

    Command autonomousCommand;
    

    //public static OI oi;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static Robot subsystem1;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    SparkMax sparkMaxstearing;
    SparkMax sparkMaxDrive;
   
    private RobotContainer m_robotContainer;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */     
    @Override
    public void robotInit() {
        CameraServer.startAutomaticCapture();

        m_robotContainer = new RobotContainer();

        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        //subsystem1 = new Subsystem1();

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // OI must be constructed after subsystems. If the OI creates Commands
        //(which it very likely will), subsystems are not guaranteed to be
        // constructed yet. Thus, their requires() statements may grab null
        // pointers. Bad news. Don't move it.
        //oi = new OI();

        // Add commands to Autonomous Sendable Chooser
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

    // An example command will be run in autonomous
    


        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS
        //SmartDashboard.putData("Auto mode", chooser);
    }

    @Override
        public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.       
        CommandScheduler.getInstance().run();
        //m_robotContainer.periodic();
    }
    /**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    @Override
    public void disabledInit(){

    }

    @Override
    public void disabledPeriodic() {
        //Scheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = m_robotContainer.getAutonomousCommand();
       
        // schedule the autonomous command (example)
        if (autonomousCommand != null){
            autonomousCommand.schedule();
            SmartDashboard.putData("auto test",autonomousCommand);
          } 
    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
    }


    @Override
    public void teleopInit() {
        
            //try (//Scheduler.getInstance().run());            
    } 
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        //if (autonomousCommand != null) autonomousCommand.cancel();


    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
       
    }

    @Override
    public void testInit() {
        //cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {

    }
}



