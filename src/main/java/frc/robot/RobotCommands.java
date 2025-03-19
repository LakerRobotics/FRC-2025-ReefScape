package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RustConstants.Arm.ArmPosition;
import frc.robot.RustConstants.Elevator.ElevatorPosition;
import frc.robot.CoralSim.CoralSimLocation;
import frc.robot.subsystems.Arm;
//import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveDriveSDS;

public class RobotCommands {
    // Tracks the last scoring level used for sequential commands
    public static ScoreLevel lastScore = ScoreLevel.None;

    /**
     * Prepares the robot for scoring at a specified level by moving the elevator and arm to the correct positions.
     * This command is used during teleop and autonomous.
     * 
     * @param level The scoring level (L1-L4)
     * @param elevator The elevator subsystem
     * @param arm The arm subsystem
     * @param coralSim The coral simulation subsystem
     * @return A command that moves the elevator and arm to the correct positions
     */
    public static Command prepareCoralScoreCommand(ScoreLevel level, Elevator elevator, Arm arm, CoralSim coralSim) {
        ElevatorPosition elevatorPosition;
        ArmPosition armPosition;
        switch (level) {
            case L1 -> {
                elevatorPosition = ElevatorPosition.L1;
                armPosition = ArmPosition.L1;
            }
            case L2 -> {
                elevatorPosition = ElevatorPosition.L2;
                armPosition = ArmPosition.L2;
            }
            case L3 -> {
                elevatorPosition = ElevatorPosition.L3;
                armPosition = ArmPosition.L3;
            }
            case L4 -> {
                elevatorPosition = ElevatorPosition.L4;
                armPosition = ArmPosition.L4;
            }
            default -> {
                throw new IllegalArgumentException("Invalid ScoreLevel");
            }
        }

        return Commands.runOnce(() -> {
            lastScore = level;
        })
                .andThen(Commands.parallel(
                        arm.moveToPositionCommand(() -> armPosition).asProxy(),
                        Commands.waitSeconds(0.5)
                                .andThen(elevator.moveToPositionCommand(() -> elevatorPosition).asProxy())));
    }

    /**
     * Similar to prepareCoralScoreCommand but optimized for autonomous routines.
     * The main difference is in the timing of the elevator and arm movements.
     * 
     * @param level The scoring level (L1-L4)
     * @param elevator The elevator subsystem
     * @param arm The arm subsystem
     * @param coralSim The coral simulation subsystem
     * @return A command that moves the elevator and arm to the correct positions
     */
    public static Command autoPrepareCoralScoreCommand(ScoreLevel level, Elevator elevator, Arm arm,
            CoralSim coralSim) {
        ElevatorPosition elevatorPosition;
        ArmPosition armPosition;
        switch (level) {
            case L1 -> {
                elevatorPosition = ElevatorPosition.L1;
                armPosition = ArmPosition.L1;
            }
            case L2 -> {
                elevatorPosition = ElevatorPosition.L2;
                armPosition = ArmPosition.L2;
            }
            case L3 -> {
                elevatorPosition = ElevatorPosition.L3;
                armPosition = ArmPosition.L3;
            }
            case L4 -> {
                elevatorPosition = ElevatorPosition.L4;
                armPosition = ArmPosition.L4;
            }
            default -> {
                throw new IllegalArgumentException("Invalid ScoreLevel");
            }
        }

        return Commands.runOnce(() -> {
            lastScore = level;
        })
                .andThen(Commands.parallel(
                        Commands.waitSeconds(0.5).andThen(arm.moveToPositionCommand(() -> armPosition)).asProxy(),
                        Commands.waitSeconds(0)
                                .andThen(elevator.moveToPositionCommand(() -> elevatorPosition).asProxy())));
    }

    /**
     * Executes the scoring action based on the last scoring level.
     * Different scoring levels have different movement patterns:
     * - L1: Moves forward and lowers elevator
     * - L2: Extends arm
     * - L3: Extends arm
     * - L4: Extends arm and lowers elevator
     * 
     * @param drivetrain The swerve drive subsystem
     * @param elevator The elevator subsystem
     * @param arm The arm subsystem
     * @param coralSim The coral simulation subsystem
     * @return A command that executes the appropriate scoring movement
     */
    public static Command scoreCoralCommand(SwerveDriveSDS drivetrain, Elevator elevator, Arm arm, CoralSim coralSim) {
        Map<ScoreLevel, Command> commandMap = Map.ofEntries(
                Map.entry(
                        ScoreLevel.L1, Commands.parallel(
                                drivetrain.moveVoltageTimeCommand(4, 0.5),
                                elevator.movePositionDeltaCommand(() -> RustConstants.Elevator.SCORING_MOVEMENT)
                                        .asProxy())),
                Map.entry(
                        ScoreLevel.L2,
                        Commands.parallel(
                                arm.movePositionDeltaCommand(() -> RustConstants.Arm.SCORING_MOVEMENT).asProxy())),
                Map.entry(
                        ScoreLevel.L3,
                        Commands.parallel(
                                arm.movePositionDeltaCommand(() -> RustConstants.Arm.SCORING_MOVEMENT).asProxy())),
                Map.entry(
                        ScoreLevel.L4,
                        Commands.parallel(
                                arm.movePositionDeltaCommand(() -> RustConstants.Arm.SCORING_MOVEMENT).asProxy(),
                                Commands.waitSeconds(0.5)
                                        .andThen(
                                                elevator.movePositionDeltaCommand(
                                                        () -> RustConstants.Elevator.SCORING_MOVEMENT))
                                        .asProxy())),
                Map.entry(
                        ScoreLevel.None,
                        Commands.none()));

        return Commands.select(commandMap, () -> lastScore);
    }

    /**
     * Prepares the robot for intake by moving the elevator and arm to the intake position.
     * This is the first step in the intake sequence.
     * 
     * @param elevator The elevator subsystem
     * @param arm The arm subsystem
     * @param coralSim The coral simulation subsystem
     * @return A command that moves the elevator and arm to intake position
     */
    public static Command prepareIntakeCoralCommand(Elevator elevator, Arm arm, CoralSim coralSim) {
        return Commands.sequence(
                Commands.parallel(elevator.moveToPositionCommand(() -> ElevatorPosition.INTAKE_PREP).asProxy(),
                        arm.moveToPositionCommand(() -> ArmPosition.BOTTOM).asProxy()));
    }

    /**
     * Executes the complete intake sequence:
     * 1. Prepares for intake
     * 2. Moves to intake position
     * 3. Activates intake mechanism
     * 4. Returns to starting position
     * 
     * @param elevator The elevator subsystem
     * @param arm The arm subsystem
     * @param coralSim The coral simulation subsystem
     * @return A command that executes the complete intake sequence
     */
    public static Command intakeCoralCommand(Elevator elevator, Arm arm, CoralSim coralSim) {
        return Commands.sequence(
                prepareIntakeCoralCommand(elevator, arm, coralSim),
                Commands.parallel(
                        elevator.moveToPositionCommand(() -> ElevatorPosition.INTAKE).asProxy(),
                        arm.moveToPositionCommand(() -> ArmPosition.BOTTOM).asProxy()),
                elevator.movePositionDeltaCommand(() -> 0.31).asProxy().alongWith(
                        Commands.waitSeconds(0.1).andThen(coralSim.setLocationCommand(CoralSimLocation.CLAW))),
                Commands.parallel(
                        Commands.waitSeconds(0.5)
                                .andThen(elevator.moveToPositionCommand(() -> ElevatorPosition.BOTTOM).asProxy()),
                        arm.moveToPositionCommand(() -> ArmPosition.TOP).asProxy()));
    }

    /**
     * Combines intake and scoring preparation into a single command.
     * Useful for autonomous routines where you want to intake and immediately prepare for scoring.
     * 
     * @param level The scoring level to prepare for after intake
     * @param elevator The elevator subsystem
     * @param arm The arm subsystem
     * @param coralSim The coral simulation subsystem
     * @return A command that intakes and prepares for scoring
     */
    public static Command intakeIntoScoreCommand(ScoreLevel level, Elevator elevator, Arm arm, CoralSim coralSim) {
        return Commands.sequence(
                Commands.parallel(
                        elevator.moveToPositionCommand(() -> ElevatorPosition.INTAKE).asProxy(),
                        arm.moveToPositionCommand(() -> ArmPosition.BOTTOM).asProxy()),
                autoPrepareCoralScoreCommand(level, elevator, arm, coralSim).alongWith(
                        Commands.waitSeconds(0.1).andThen(coralSim.setLocationCommand(CoralSimLocation.CLAW))));
    }

    /**
     * Prepares the robot for removing algae from Level 2.
     * Moves the elevator and arm to the correct position for algae removal.
     * 
     * @param elevator The elevator subsystem
     * @param arm The arm subsystem
     * @return A command that moves the elevator and arm to L2 algae removal position
     */
    public static Command prepareAlgaeL2RemoveCommand(Elevator elevator, Arm arm) {
        return Commands.sequence(
                Commands.parallel(elevator.moveToPositionCommand(() -> ElevatorPosition.ALGAE_L2).asProxy(),
                        arm.moveToPositionCommand(() -> ArmPosition.HORIZONTAL).asProxy()));
    }

    /**
     * Prepares the robot for removing algae from Level 3.
     * Moves the elevator and arm to the correct position for algae removal.
     * 
     * @param elevator The elevator subsystem
     * @param arm The arm subsystem
     * @return A command that moves the elevator and arm to L3 algae removal position
     */
    public static Command prepareAlgaeL3RemoveCommand(Elevator elevator, Arm arm) {
        return Commands.sequence(
                Commands.parallel(elevator.moveToPositionCommand(() -> ElevatorPosition.ALGAE_L3).asProxy(),
                        arm.moveToPositionCommand(() -> ArmPosition.HORIZONTAL).asProxy()));
    }

    /**
     * Executes the algae removal action.
     * Moves the robot backwards slightly while lowering the elevator to remove algae.
     * 
     * @param drivetrain The swerve drive subsystem
     * @param elevator The elevator subsystem
     * @param arm The arm subsystem
     * @return A command that executes the algae removal movement
     */
    public static Command algaeRemoveCommand(SwerveDriveSDS drivetrain, Elevator elevator, Arm arm) {
        return Commands.sequence(
                Commands.parallel(
                        drivetrain.moveVoltageTimeCommand(-2, 0.5),
                        elevator.movePositionDeltaCommand(() -> -0.06).asProxy()));
    }
}