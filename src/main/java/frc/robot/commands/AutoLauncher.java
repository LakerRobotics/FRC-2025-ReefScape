// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands; 

import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

  public class AutoLauncher extends SequentialCommandGroup{

  public AutoLauncher (LauncherSubsystem mLauncherSubsystem) {
    
    // Spin up the Launcher
    addCommands( new LauncherRun(mLauncherSubsystem).withTimeout(2));

   
  }
  
}
