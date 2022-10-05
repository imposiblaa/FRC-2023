// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.swerveModConstants;
import frc.robot.Constants.swerveModConstants.driveConstants;

public class SwerveModSubsystem extends SubsystemBase {

  private final PIDController turningPID;

  //private final turningMotor;
  private final TalonFX driveMotor;
  
  public SwerveModSubsystem(int drivingID, int turningID) {

    driveMotor = new TalonFX(drivingID);

    turningPID = new PIDController(swerveModConstants.kTurningP, swerveModConstants.kTurningI, swerveModConstants.kTurningD);
    turningPID.enableContinuousInput(-Math.PI, Math.PI);

  }

  private Rotation2d getAngle() {
    double angle = 0;
    return new Rotation2d(angle);
  }


  public void setState(SwerveModuleState targetState) {
    targetState = SwerveModuleState.optimize(targetState, getAngle());
    driveMotor.set(TalonFXControlMode.PercentOutput, targetState.speedMetersPerSecond / driveConstants.kMaxSpeedMPS);
    //set turning velocity(turningPID.calculate([turning encoder position], targetState.angle.getRadians))

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
