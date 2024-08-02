package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.TimestampedDoubleArray;

public class PoseEstimate {

    public final Pose2d pose;
    public final double timestampSeconds;
    public final double latency;
    public final int tagCount;
    public final double tagSpan;
    public final double averageTagDistance;
    public final double averageTagArea;

    public PoseEstimate(TimestampedDoubleArray entry) {
        double [] array = entry.value;
        this.pose = getPose(array);
        this.latency = getDouble(array, 6);
        this.timestampSeconds = (entry.timestamp / 1000000.0) - (latency / 1000.0);
        this.tagCount = (int) getDouble(array, 7);
        this.tagSpan = getDouble(array, 8);
        this.averageTagDistance = getDouble(array, 9);
        this.averageTagArea = getDouble(array, 10);
    }

    private Pose2d getPose(double [] array) {
        double tx = array[0];
        double ty = array[1];
        double yd = array[5];
        return new Pose2d(new Translation2d(tx, ty), Rotation2d.fromDegrees(yd));
    }

    private double getDouble(double [] array, int idx) {
        return array.length > idx ? array[idx] : 0.0;
    }
}
