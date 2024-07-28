package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.shooter.ShooterTuningCommand;
import frc.robot.subsystems.arm2.ArmHardwareSim;
import frc.robot.subsystems.arm2.ArmSubsystem;
import frc.robot.subsystems.shooter.ShooterHardwareSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private ArmSubsystem arm;
  private ShooterSubsystem shooter;
  private XboxController controller;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    controller = new XboxController(0);

    arm = new ArmSubsystem(new ArmHardwareSim());
    arm.setDefaultCommand(new ArmTuningCommand(arm));

    shooter = new ShooterSubsystem(new ShooterHardwareSim());
    shooter.setDefaultCommand(new ShooterTuningCommand(shooter));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopExit() {
    arm.release();
    shooter.release();
  }
}
