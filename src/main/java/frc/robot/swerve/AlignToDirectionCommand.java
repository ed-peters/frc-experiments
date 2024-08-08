package frc.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.SmarterDashboard;

/**
 * Command to align the robot to a fixed heading.
 */
public class AlignToDirectionCommand extends Command {

    public static final double DEFAULT_KP = 0.0;
    public static final double THRESHOLD = Units.degreesToRadians(2.0);

    private final SwerveDriveSubsystem drive;
    private final PIDController pid;
    private final ChassisSpeeds speeds;
    private final double targetRadians;
    private double currentRadians;
    private boolean active;

    public AlignToDirectionCommand(SwerveDriveSubsystem drive, double targetRadians) {

        this.drive = drive;
        this.pid = new PIDController(DEFAULT_KP, 0.0, 0.0);
        this.speeds = new ChassisSpeeds();
        this.targetRadians = targetRadians;
        this.currentRadians = Double.NaN;
        this.active = false;

        SmarterDashboard.putData("AlignToDirectionCommand-"+Units.degreesToRadians(targetRadians), builder -> {
            builder.addBoolean("Active?", () -> active);
            builder.displayAsDegrees("CurrentHeading", () -> currentRadians);
            builder.displayAsDegrees("CurrentHeading", () -> currentRadians);
            builder.addDouble("kP", pid::getP, pid::setP);
            builder.displayAsDegrees("TargetHeading", () -> targetRadians);
            builder.displayAsDegrees("Speed", () -> speeds.omegaRadiansPerSecond);
        });

    }

    @Override
    public void initialize() {
        active = true;
    }

    @Override
    public void execute() {

        currentRadians = drive.getPose().getRotation().getDegrees();
        if (Math.abs(targetRadians - currentRadians) < THRESHOLD) {
            end(false);
            return;
        }

        speeds.omegaRadiansPerSecond = pid.calculate(currentRadians, targetRadians);
        drive.driveRobotRelative(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        speeds.omegaRadiansPerSecond = 0.0;
        currentRadians = Double.NaN;
        active = false;
        drive.driveRobotRelative(speeds);
    }
}
