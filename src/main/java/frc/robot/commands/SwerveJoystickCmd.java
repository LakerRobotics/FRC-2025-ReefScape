package frc.robot.commands;
  
import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants2023.Swerve;
import frc.robot.Constants2023;
import frc.robot.subsystems.SwerveDriveSDS;
  
public class SwerveJoystickCmd extends Command {

    private final SwerveDriveSDS swerveSubsystem;
    private final java.util.function.Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private SlewRateLimiter xLimiter;
    private SlewRateLimiter yLimiter;
    private SlewRateLimiter turningLimiter;

    public SwerveJoystickCmd(SwerveDriveSDS swervesubsystem2,
            Supplier<Double> xSpdFunction, 
            Supplier<Double> ySpdFunction, 
            Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction) {
        this.swerveSubsystem = swervesubsystem2;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(Swerve.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(Swerve.kTeleDriveAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(Swerve.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swervesubsystem2);
        
    } 
/* 
        public SwerveJoystickCmd () {

        }
*/
        @Override 
        public void initialize () {

        }
        double temp =0;
     
        @Override 
        public void execute () {
            // 1. Get real-time joystick inputs
            double xSpeed = xSpdFunction.get();
            double ySpeed = ySpdFunction.get();
            double turningSpeed = turningSpdFunction.get();
SmartDashboard.putNumber("xSpeed", xSpeed);
SmartDashboard.putNumber("ySpeed", ySpeed);
SmartDashboard.putNumber("turningSpeed", turningSpeed);

temp = temp+0.1;
SmartDashboard.putNumber("Test SwerveJoysticCmd", temp);


            //2. Applu deadband
            xSpeed = Math.abs(xSpeed) > Swerve.kDeadband ? xSpeed : 0.0;
            ySpeed = Math.abs(ySpeed) > Swerve.kDeadband ? ySpeed : 0.0;
            turningSpeed = Math.abs(turningSpeed) > Swerve.kDeadband ? turningSpeed : 0.0;

        }

        @Override 
        public void end(boolean interrupted) {

        }

        @Override 
        public boolean isFinished() {
            return false;
        }
}