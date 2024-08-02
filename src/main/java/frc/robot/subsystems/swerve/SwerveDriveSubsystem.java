package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.vision.PoseEstimate;

/**
 * Not a full implementation of a swerve drive - this just captures a few
 * methods that we'd need for integrated vision processing.
 */
public interface SwerveDriveSubsystem extends Subsystem {

    ChassisSpeeds STOP = new ChassisSpeeds();

    Pose2d getPose();

    void acceptPoseEstimate(PoseEstimate pose);

    void driveRobotRelative(ChassisSpeeds speeds);

    default void stop() {
        driveRobotRelative(STOP);
    }
}
