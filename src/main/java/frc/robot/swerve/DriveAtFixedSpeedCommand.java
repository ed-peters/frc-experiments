package frc.robot.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.SmarterDashboard;

public class DriveAtFixedSpeedCommand extends Command {

    public static final double DEFAULT_DRIVE_TIME = 1.0;
    public static final ChassisSpeeds DEFAULT_DRIVE_SPEED = new ChassisSpeeds(
            Units.feetToMeters(3.0),
            0.0,
            0.0);

    private final SwerveDriveSubsystem drive;
    private double driveTime;
    private ChassisSpeeds driveSpeed;
    private double startTime;

    public DriveAtFixedSpeedCommand(String name, SwerveDriveSubsystem drive) {

        this.drive = drive;
        this.driveTime = DEFAULT_DRIVE_TIME;
        this.driveSpeed = DEFAULT_DRIVE_SPEED;
        this.startTime = Double.NaN;

        addRequirements(drive);

        SmarterDashboard.putData(name, builder -> {
            builder.displayAsDegrees("Tuning/DriveSpeedOmega", () -> driveSpeed.omegaRadiansPerSecond, val -> driveSpeed.omegaRadiansPerSecond = val);
            builder.displayAsFeet("Tuning/DriveSpeedX", () -> driveSpeed.vxMetersPerSecond, val -> driveSpeed.vxMetersPerSecond = val);
            builder.displayAsFeet("Tuning/DriveSpeedY", () -> driveSpeed.vyMetersPerSecond, val -> driveSpeed.vyMetersPerSecond = val);
            builder.addDouble("Tuning/DriveTime", () -> driveTime, val -> driveTime = val);
            builder.addDouble("Output/StartTime", () -> startTime, val -> startTime = val);
        });
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        drive.driveRobotRelative(driveSpeed);
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime > driveTime;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
