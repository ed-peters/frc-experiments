package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

public class LimelightSubsystem extends SubsystemBase {

    public static final double [] NO_DATA = new double[0];

    private final DoubleEntry currentPipeline;
    private final DoubleEntry horizontalOffset;
    private final DoubleEntry verticalOffset;
    private final DoubleEntry targetArea;
    private final DoubleEntry targetCount;
    private final DoubleArrayEntry megaTag;
    private final Supplier<Pose2d> fieldPoseSupplier;
    private final DoubleArrayEntry fieldPose;

    public LimelightSubsystem(Supplier<Pose2d> poseSupplier) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        currentPipeline = table.getDoubleTopic("pipeline").getEntry(-1.0);
        targetArea = table.getDoubleTopic("ta").getEntry(0.0);
        horizontalOffset = table.getDoubleTopic("tx").getEntry(0.0);
        verticalOffset = table.getDoubleTopic("ty").getEntry(0.0);
        targetCount = table.getDoubleTopic("tv").getEntry(0.0);
        megaTag = table.getDoubleArrayTopic("botpose_orb_wpiblue").getEntry(NO_DATA);
        fieldPose = table.getDoubleArrayTopic("robot_orientation_set").getEntry(NO_DATA);
        fieldPoseSupplier = poseSupplier;
    }

    public void setPipeline(LimelightPipeline desiredPipeline) {
        currentPipeline.set(desiredPipeline.id);
    }

    public LimelightPipeline getCurrentPipeline() {
        return LimelightPipeline.forId((int)currentPipeline.get());
    }

    public void updateTargetInfo(TargetInfo info) {
        if (targetCount.get() > 0.0) {
            info.setTarget(
                    horizontalOffset.get(),
                    verticalOffset.get(),
                    targetArea.get());
        } else {
            info.setNoTarget();
        }
    }

    public PoseEstimate getPoseEstimate() {
        TimestampedDoubleArray data = megaTag.getAtomic();
        if (data == null || data.value == null || data.value.length < 6) {
            return null;
        }
        return new PoseEstimate(data);
    }

    @Override
    public void periodic() {

        if (fieldPoseSupplier != null) {
            Pose2d pose = fieldPoseSupplier.get();
            if (pose != null) {
                fieldPose.set(new double[]{
                        pose.getRotation().getDegrees(),
                        0.0, 0.0, 0.0, 0.0, 0.0
                });
            }
        }

    }
}
