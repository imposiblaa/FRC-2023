package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveBaseSubsystem;

public class TestCmd extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private SwerveBaseSubsystem swerveBaseSubsystem;

    public TestCmd(SwerveBaseSubsystem swerveBaseSubsystem) {
        this.swerveBaseSubsystem = swerveBaseSubsystem;
    }

    @Override
    public void execute() {
        double xAxis = RobotContainer.driveController.getRawAxis(0);
        SwerveModuleState[] modStates = new SwerveModuleState[4];
        for(int i=0; i<4; i++){
            modStates[i] = new SwerveModuleState(0, new Rotation2d(xAxis*360));
        }
        swerveBaseSubsystem.setSwerveState(modStates);
    }
}