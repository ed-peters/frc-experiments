package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.arm.ArmTeleopCommand;
import frc.robot.commands.limelight.AprilTagPoseEstimateCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.subsystems.wheel.WheelSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    private XboxController controller;
    private ArmSubsystem arm;
    private WheelSubsystem shooter;
    private WheelSubsystem intake;
    private LimelightSubsystem limelight;
    private SwerveDriveSubsystem drive;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {

        controller = new XboxController(0);

        arm = new ArmSubsystem(3, 4);
        arm.setDefaultCommand(new ArmTeleopCommand(arm, () -> -controller.getLeftY()));

        shooter = new WheelSubsystem(1);
        shooter.setDefaultCommand(shooter.stopCommand());

        intake = new WheelSubsystem(2);
        intake.setDefaultCommand(intake.stopCommand());

        limelight = new LimelightSubsystem(drive::getPose);
        limelight.setDefaultCommand(new AprilTagPoseEstimateCommand(limelight, drive));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopExit() {
    }
}
