// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveSDS;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/** An example command that uses an example subsystem. */
public class SysIdRoutineBotQReverse extends Command {
  
  private final SwerveDriveSDS m_driveTrain;
 
  /**
 
   *
   * @param subsystem The subsystem used by this command.
   */
  public SysIdRoutineBotQReverse(SwerveDriveSDS driveSDS) {
    m_driveTrain = driveSDS;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSDS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
