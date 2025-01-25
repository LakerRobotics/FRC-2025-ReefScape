// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ArmJoystickControl extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_subsystem;
 
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmJoystickControl(ArmSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_subsystem.isLimitSwitchTopPressed() == false){
    
      //see if past the zero (positive) and if so reset this as the new zero
      if(m_subsystem.getArmPosition()>0){
        m_subsystem.setCanSparkEncoderToZero();
        m_subsystem.runManual(-0.2);
      }
      else{
        m_subsystem.setTargetPosition(Constants.Arm.kHomePosition);
      }
    }
      // continue running the arm up till hits the limit swithc (the home position)
    else{
        m_subsystem.runManual(0);
        // this is redudent but just to help document and understand hitting the limit switch will put power to zero
    }
   
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
