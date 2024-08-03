package frc.robot.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.limelight.PoseEstimate;
import frc.robot.util.SmarterDashboard;

public class MockSwerveDriveSubsystem extends SubsystemBase implements SwerveDriveSubsystem {

    private Pose2d lastPose;
    private ChassisSpeeds lastSpeed;

    public MockSwerveDriveSubsystem() {

        lastPose = new Pose2d();
        lastSpeed = STOP;

        SmarterDashboard.putData("MockSwerveDriveSubsystem", builder -> {
            builder.addPose("Pose", () -> lastPose);
            builder.addSpeeds("Speed", () -> lastSpeed);
        });
    }

    @Override
    public Pose2d getPose() {
        return lastPose;
    }

    @Override
    public void acceptPoseEstimate(PoseEstimate pose) {
        lastPose = pose.pose;
    }

    @Override
    public void driveRobotRelative(ChassisSpeeds speeds) {
    }

    @Override
    public void periodic() {
        if (lastSpeed != STOP) {
            double x = lastPose.getX() + lastSpeed.vxMetersPerSecond * 0.02;
            double y = lastPose.getY() + lastSpeed.vyMetersPerSecond * 0.02;
            double r = lastPose.getRotation().getRadians() + lastSpeed.omegaRadiansPerSecond * 0.02;
            lastPose = new Pose2d(x, y, Rotation2d.fromDegrees(r));
        }
    }
}
