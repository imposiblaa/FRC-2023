// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.swerveModConstants.driveConstants;
import frc.robot.subsystems.SwerveBaseSubsystem;
import frc.robot.subsystems.SwerveModSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class SwerveAbsCmd extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final SwerveBaseSubsystem m_swerveBaseSubsystem;

  private final Translation2d m_frontLeftLoc;
  private final Translation2d m_backLeftLoc;
  private final Translation2d m_frontRightLoc;
  private final Translation2d m_backRightLoc;

  private final SwerveDriveKinematics m_swerveKinematics;

  

  public SwerveAbsCmd(SwerveBaseSubsystem swerveBaseSubsystem) {

    this.m_swerveBaseSubsystem = swerveBaseSubsystem;

    m_frontLeftLoc = new Translation2d(-driveConstants.kTrack/2, driveConstants.kWheelBase/2);
    m_backLeftLoc = new Translation2d(driveConstants.kTrack/-2, driveConstants.kWheelBase/2);
    m_frontRightLoc = new Translation2d(driveConstants.kTrack/2, driveConstants.kWheelBase/-2);
    m_backRightLoc = new Translation2d(driveConstants.kTrack/-2, driveConstants.kWheelBase/-2);

    m_swerveKinematics = new SwerveDriveKinematics(
      m_frontLeftLoc, m_frontRightLoc, m_backLeftLoc, m_backRightLoc
    );

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xAxis = RobotContainer.driveController.getRawAxis(0);
    double yAxis = RobotContainer.driveController.getRawAxis(1);
    double thetaAxis = RobotContainer.driveController.getRawAxis(4);


    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      xAxis*driveConstants.kSpeedMultiplier, yAxis*driveConstants.kSpeedMultiplier, thetaAxis*driveConstants.kSpeedMultiplier,
      m_swerveBaseSubsystem.getBaseAngle()
    );

    SwerveModuleState[] targetStates = m_swerveKinematics.toSwerveModuleStates(speeds);
    m_swerveBaseSubsystem.setSwerveState(targetStates);

  }

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
