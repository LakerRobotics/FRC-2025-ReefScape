// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;  

import edu.wpi.first.math.geometry.Pose2d; 
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Swerve.ModulePosition;
import frc.robot.subsystems.SwerveDriveSDS;
//import frc.robot.subsystems.SwerveDriveRev;
import frc.robot.utils.ModuleMap;

import java.util.Map;



public class FieldSim {
  private final SwerveDriveSDS m_swerveDrive;

  private final Field2d m_field2d = new Field2d();

  private Pose2d[] m_swerveModulePoses = {
          new Pose2d(),
          new Pose2d(),
          new Pose2d(),
          new Pose2d()
  };

  public FieldSim(SwerveDriveSDS swerveDrive) {
    m_swerveDrive = swerveDrive;
  }

  public void initSim() {}

  public Field2d getField2d() {
    return m_field2d;
  }

  private void updateRobotPoses() {
    m_field2d.setRobotPose(m_swerveDrive.getPoseMeters());

    for (int i = 0; i < Swerve.kModuleTranslations.length; i++) {
//      Translation2d updatedPositions =
//              Swerve.kModuleTranslations[i]
//                      .rotateBy(m_swerveDrive.getPoseMeters().getRotation())
//                      .plus(m_swerveDrive.getPoseMeters().getTranslation());
//      m_swerveModulePoses[i] =
//              new Pose2d(
//                      updatedPositions,
//                      m_swerveDrive.
//                              .getSwerveModule(i)
//                              .getHeadingRotation2d()
//                              .plus(m_swerveDrive.getHeadingRotation2d())
//                        );
    }

    m_field2d.getObject("Swerve Modules").setPoses(m_swerveModulePoses);
  }

  public void periodic() {
    updateRobotPoses();

    if (RobotBase.isSimulation()) simulationPeriodic();

    SmartDashboard.putData("Field2d", m_field2d);
  }

  public void simulationPeriodic() {}
}