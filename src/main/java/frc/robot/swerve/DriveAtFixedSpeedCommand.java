package frc.robot.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.SmarterDashboard;

/**
 * Command to drive the robot at a fixed speed. Use e.g. {@link Command#withTimeout(double)}
 * to restrict this to run for a specific amount of time.
 */
public class DriveAtFixedSpeedCommand extends Command {

    private final SwerveDriveSubsystem drive;
    private final ChassisSpeeds driveSpeed;
    private boolean active;

    public DriveAtFixedSpeedCommand(String name, SwerveDriveSubsystem drive, ChassisSpeeds driveSpeed) {

        this.drive = drive;
        this.driveSpeed = driveSpeed;
        this.active = false;

        addRequirements(drive);

        SmarterDashboard.putData(name, builder -> {
            builder.addBoolean("Active?", () -> active);
            builder.displayAsDegrees("SpeedOmega", () -> driveSpeed.omegaRadiansPerSecond);
            builder.displayAsFeet("SpeedX", () -> driveSpeed.vxMetersPerSecond);
            builder.displayAsFeet("SpeedY", () -> driveSpeed.vyMetersPerSecond);
        });
    }

    @Override
    public void initialize() {
        active = true;
    }

    @Override
    public void execute() {
        drive.driveRobotRelative(driveSpeed);
    }

    @Override
    public boolean isFinished() {
        return active;
    }

    @Override
    public void end(boolean interrupted) {
        active = false;
        drive.stop();
    }
}
