package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveDriveSDS;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoShootSpeakerThenFollowPath extends SequentialCommandGroup {

    public AutoShootSpeakerThenFollowPath(ArmSubsystem armSubsystem, LauncherSubsystem launcherSubsystem, IntakeSubsystem intakeSubsystem, SwerveDriveSDS driveTrain, String pathName) {
        // Ensure the arm is in the home position
        addCommands(new ArmJoystickControl(armSubsystem).withTimeout(1));

        // Spin up the Launcher
        addCommands(new LauncherRun(launcherSubsystem).withTimeout(2));

        // Setup the command to run the launcher and intake simultaneously
        ParallelCommandGroup runLauncherAndIntake = new ParallelCommandGroup(
            new LauncherRun(launcherSubsystem).withTimeout(2),
            new IntakeSetPower(intakeSubsystem, 1).withTimeout(2)
        );

        // Add the parallel group to the sequence
        addCommands(runLauncherAndIntake);

        // Follow the specified path
        addCommands(new DriveTrainFollowPath(driveTrain, pathName).withTimeout(2));
    }
}
