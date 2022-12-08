// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.swerveModConstants;
import frc.robot.Constants.swerveModConstants.driveConstants;

public class SwerveModSubsystem extends SubsystemBase {

  private final PIDController turningPID;

  //private final turningMotor;
  private final TalonFX driveMotor;

  private final TalonSRX turningMotor;

  private final Encoder relativeEncoder;
  
  public SwerveModSubsystem(int drivingID, int turningID, int[] relIDs) {

    driveMotor = new TalonFX(drivingID);

    turningMotor = new TalonSRX(turningID);

    relativeEncoder = new Encoder(relIDs[0], relIDs[1]);
    relativeEncoder.setDistancePerPulse(Math.PI*swerveModConstants.kCPR);
    relativeEncoder.reset();

    turningPID = new PIDController(swerveModConstants.kTurningP,swerveModConstants.kTurningI, swerveModConstants.kTurningD);
    turningPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  private Rotation2d getAngle() {
    double angle = relativeEncoder.getDistance();
    //System.out.print("\n" + angle);
    return new Rotation2d(angle);
  }


  public void setState(SwerveModuleState targetState) {
    targetState = SwerveModuleState.optimize(targetState, getAngle());
    driveMotor.set(TalonFXControlMode.PercentOutput, targetState.speedMetersPerSecond / driveConstants.kMaxSpeedMPS);
    turningMotor.set(TalonSRXControlMode.PercentOutput, turningPID.calculate(getAngle().getRadians(), targetState.angle.getRadians()));

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
