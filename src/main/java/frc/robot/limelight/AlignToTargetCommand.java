package frc.robot.limelight;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.SwerveDriveSubsystem;
import frc.robot.swerve.DriveAtFixedSpeedCommand;
import frc.robot.util.SmarterDashboard;

/**
 * Uses the {@link LimelightSubsystem} to get target information and the {@link SwerveDriveSubsystem}
 * to spin the robot until the target is within a certain range of the center of the field of view.
 *
 * Combine this with a {@link DriveAtFixedSpeedCommand} to implement a
 * behavior of orienting on a target and then driving towards it.
 */
public class AlignToTargetCommand extends Command {

    public static final double DEFAULT_PIPELINE = 1;
    public static final double DEFAULT_TARGETING_FACTOR = 5.0;
    public static final double DEFAULT_TARGETING_MAX_SPEED = 5.0;
    public static final double DEFAULT_TARGETING_TOLERANCE = 2.0;

    private final LimelightSubsystem limelight;
    private final SwerveDriveSubsystem drive;
    private final TargetInfo target;
    private final ChassisSpeeds speeds;
    private final SendableChooser<LimelightPipeline> pipeline;
    private double speedFactor;
    private double maxSpeed;
    private double tolerance;
    private boolean done;

    public AlignToTargetCommand(LimelightSubsystem limelight, SwerveDriveSubsystem drive) {

        this.limelight = limelight;
        this.drive = drive;
        this.speedFactor = DEFAULT_TARGETING_FACTOR;
        this.maxSpeed = DEFAULT_TARGETING_MAX_SPEED;
        this.tolerance = DEFAULT_TARGETING_TOLERANCE;
        this.pipeline = LimelightPipeline.makeChooser();
        this.target = new TargetInfo();
        this.speeds = new ChassisSpeeds();
        this.done = false;

        addRequirements(limelight, drive);

        SmarterDashboard.putChooser("AlignToTargetCommand/Pipeline", pipeline);
        SmarterDashboard.putData("AlignToTargetCommand", builder -> {
            builder.addBoolean("Done?", () -> done);
            builder.displayAsDegrees("Tuning/MaxSpeed", () -> maxSpeed, val -> maxSpeed = val);
            builder.addDouble("Tuning/SpeedFactor", () -> speedFactor, val -> speedFactor = val);
            builder.addDouble("Tuning/Tolerance", () -> tolerance, val -> tolerance = val);
            builder.addBoolean("Output/HasTarget?", () -> target.hasTarget);
            builder.addDouble("Output/HorizontalOffset", () -> target.horizontalOffset);
            builder.displayAsDegrees("Output/Speed", () -> speeds.omegaRadiansPerSecond);
        });
    }

    @Override
    public void initialize() {
        limelight.setPipeline(pipeline.getSelected());
        target.setNoTarget();
        speeds.omegaRadiansPerSecond = 0.0;
        done = false;
    }

    @Override
    public void execute() {
        limelight.updateTargetInfo(target);
        if (target.hasTarget && Math.abs(target.horizontalOffset) > tolerance) {
            speeds.omegaRadiansPerSecond = speedFactor * target.horizontalOffset;
            drive.driveRobotRelative(speeds);
        } else {
            done = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        speeds.omegaRadiansPerSecond = 0.0;
        target.setNoTarget();
        done = true;
    }
}
