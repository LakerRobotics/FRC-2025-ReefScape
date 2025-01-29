package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.SwerveDriveREVReal;
import java.nio.file.Path;
import java.io.IOException;
import edu.wpi.first.wpilibj.Timer;

public class DriveTrainFollowPath extends Command {
    private final SwerveDriveREVReal driveSubsystem;
    private Trajectory trajectory;
    private final String trajectoryJSON;
    private final Timer timer = new Timer();

    public DriveTrainFollowPath(SwerveDriveREVReal subsystem, String trajectoryPath) {
        this.driveSubsystem = subsystem;
        this.trajectoryJSON = trajectoryPath;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        try {
            Path trajectoryFilePath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryFilePath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
        driveSubsystem.resetPose(trajectory.getInitialPose());
    }

    @Override
    public void execute() {
        var timeSinceInit = timer.get(); 
        if (timeSinceInit < trajectory.getTotalTimeSeconds()) {
            var desiredPose = trajectory.sample(timeSinceInit);
            driveSubsystem.followTrajectory(desiredPose);
        }
    }

    @Override
    public boolean isFinished() {
        return timeSinceInitialized() >= trajectory.getTotalTimeSeconds();
    }

    private double timeSinceInitialized() {
        return timer.get();
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        driveSubsystem.drive(0,0,0,true,false);
    }
}
