package frc.robot.limelight;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.SwerveDriveSubsystem;
import frc.robot.util.SmarterDashboard;

public class AprilTagPoseEstimateCommand extends Command {

    private final LimelightSubsystem limelight;
    private final SwerveDriveSubsystem drive;
    private final SendableChooser<LimelightPipeline> pipeline;
    private PoseEstimate lastPose;
    public boolean log;


    public AprilTagPoseEstimateCommand(LimelightSubsystem limelight, SwerveDriveSubsystem drive) {

        this.limelight = limelight;
        this.drive = drive;
        this.pipeline = LimelightPipeline.makeChooser();
        this.lastPose = null;

        SmarterDashboard.putChooser("AprilTagPoseEstimateCommand/Pipeline", pipeline);
        SmarterDashboard.putData("AprilTagPoseEstimateCommand", builder -> {
            builder.addBoolean("HasPose?", () -> lastPose != null);
            builder.addPose("LastPose", () -> lastPose == null ? null : lastPose.pose);
            builder.addBoolean("Log?", () -> log, val -> log = val);
        });
    }

    @Override
    public void initialize() {
        lastPose = null;
        limelight.setPipeline(pipeline.getSelected());
    }

    @Override
    public void execute() {

        lastPose = limelight.getPoseEstimate();
        if (lastPose == null) {
            return;
        }

        if (log) {
            System.out.printf("Pose Estimate Information:%n");
            System.out.printf("Timestamp (Seconds): %.3f%n", lastPose.timestampSeconds);
            System.out.printf("Latency: %.3f ms%n", lastPose.latency);
            System.out.printf("Tag Count: %d%n", lastPose.tagCount);
            System.out.printf("Tag Span: %.2f meters%n", lastPose.tagSpan);
            System.out.printf("Average Tag Distance: %.2f meters%n", lastPose.averageTagDistance);
            System.out.printf("Average Tag Area: %.2f%% of image%n", lastPose.averageTagArea);
            System.out.println();
        }

        drive.acceptPoseEstimate(lastPose);
    }

    @Override
    public void end(boolean interrupted) {
        lastPose = null;
    }
}
