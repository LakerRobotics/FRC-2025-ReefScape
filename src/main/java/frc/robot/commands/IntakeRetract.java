package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRetract extends Command {
    private final IntakeSubsystem intakeSubsystem;

    public IntakeRetract(IntakeSubsystem subsystem) {
        this.intakeSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(this.intakeSubsystem);
    }

    @Override
    public void initialize() {
        // Initialization code here, if any.
    }

    @Override
    public void execute() {
        // Code to retract the arm.
        intakeSubsystem.retract();
    }

    @Override
    public boolean isFinished() {
        // You can add logic here to determine when the command is finished.
        // For now, we'll just return true so it runs once and exits.
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        // Code to run when the command ends or is interrupted.
        intakeSubsystem.setPower(0);
    }
}

