package frc.robot.subsystems;

import org.opencv.core.RotatedRect;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.swerveModConstants;
import frc.robot.Constants.swerveModConstants.driveConstants;

public class SwerveBaseSubsystem extends SubsystemBase {

    private final SwerveModSubsystem frontLeftMod = new SwerveModSubsystem(
        swerveModConstants.mod1.drivingMotorID, swerveModConstants.mod1.turningMotorID
    );

    private final SwerveModSubsystem backLeftMod = new SwerveModSubsystem(
        swerveModConstants.mod2.drivingMotorID, swerveModConstants.mod2.turningMotorID
    );

    private final SwerveModSubsystem frontRightMod = new SwerveModSubsystem(
        swerveModConstants.mod3.drivingMotorID, swerveModConstants.mod3.turningMotorID
    );

    private final SwerveModSubsystem backRightMod = new SwerveModSubsystem(
        swerveModConstants.mod4.drivingMotorID, swerveModConstants.mod4.turningMotorID
    );


    public SwerveBaseSubsystem() {

    }

    public Rotation2d getBaseAngle() {
        Rotation2d angle = new Rotation2d(0);
        return angle;
    }

    public void setSwerveState(SwerveModuleState[] targetStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, driveConstants.kMaxSpeedMPS);
        frontLeftMod.setState(targetStates[0]);
        backLeftMod.setState(targetStates[1]);
        frontRightMod.setState(targetStates[2]);
        backRightMod.setState(targetStates[3]);
    }
}