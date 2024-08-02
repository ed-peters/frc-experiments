package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.LimelightPipeline;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.subsystems.vision.PoseEstimate;
import frc.robot.util.SmarterDashboard;

public class AprilTagPoseEstimateCommand extends Command {

    private final LimelightSubsystem limelight;
    private final SwerveDriveSubsystem drive;
    private final SendableChooser<LimelightPipeline> pipeline;
    public boolean hasPose;
    public boolean log;


    public AprilTagPoseEstimateCommand(LimelightSubsystem limelight, SwerveDriveSubsystem drive) {

        this.limelight = limelight;
        this.drive = drive;
        this.pipeline = LimelightPipeline.makeChooser();

        SmarterDashboard.putChooser("AprilTagPoseEstimateCommand/Pipeline", pipeline);
        SmarterDashboard.putData("AprilTagPoseEstimateCommand", builder -> {
            builder.addBoolean("HasPose?", () -> hasPose);
            builder.addBoolean("Log?", () -> log, val -> log = val);
        });
    }

    @Override
    public void initialize() {
        hasPose = false;
        limelight.setPipeline(pipeline.getSelected());
    }

    @Override
    public void execute() {

        PoseEstimate pose = limelight.getPoseEstimate();
        if (pose == null) {
            hasPose = false;
            return;
        }

        hasPose = true;
        if (log) {
            System.out.printf("Pose Estimate Information:%n");
            System.out.printf("Timestamp (Seconds): %.3f%n", pose.timestampSeconds);
            System.out.printf("Latency: %.3f ms%n", pose.latency);
            System.out.printf("Tag Count: %d%n", pose.tagCount);
            System.out.printf("Tag Span: %.2f meters%n", pose.tagSpan);
            System.out.printf("Average Tag Distance: %.2f meters%n", pose.averageTagDistance);
            System.out.printf("Average Tag Area: %.2f%% of image%n", pose.averageTagArea);
            System.out.println();
        }

        drive.acceptPoseEstimate(pose);
    }

    @Override
    public void end(boolean interrupted) {
        hasPose = false;
    }
}
