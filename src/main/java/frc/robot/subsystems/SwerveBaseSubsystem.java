package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.swerveModConstants;
import frc.robot.Constants.swerveModConstants.driveConstants;

public class SwerveBaseSubsystem extends SubsystemBase {

    private final SwerveModSubsystem frontLeftMod = new SwerveModSubsystem(

    );

    private final SwerveModSubsystem backLeftMod = new SwerveModSubsystem(

    );

    private final SwerveModSubsystem frontRightMod = new SwerveModSubsystem(

    );

    private final SwerveModSubsystem backRightMod = new SwerveModSubsystem(

    );


    public SwerveBaseSubsystem() {

    }

    public void setSwerveState(SwerveModuleState[] targetStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, driveConstants.kMaxSpeedMPS);
    }
}