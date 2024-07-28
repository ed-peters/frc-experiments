package frc.robot.commands.wheel;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wheel.WheelSubsystem;
import frc.robot.util.SmarterDashboard;

/**
 * Command for use in tuning the {@link WheelSubsystem}
 */
public class WheelTuningCommand extends Command {

    public static final double DEFAULT_REVOLUTIONS_PER_MINUTE = 250;

    private final WheelSubsystem wheel;
    private double setpointRevolutionsPerMinute;
    private boolean isEnabled;

    public WheelTuningCommand(WheelSubsystem wheel) {

        this.wheel = wheel;
        this.setpointRevolutionsPerMinute = DEFAULT_REVOLUTIONS_PER_MINUTE;
        this.isEnabled = false;

        addRequirements(wheel);

        SmarterDashboard.putData("ShooterTuningCommand", builder -> {
            builder.addBoolean("Enabled?", () -> isEnabled, this::setEnabled);
            builder.addDouble("kP", wheel::getP, wheel::setP);
            builder.addDouble("kV", wheel::getV, wheel::setV);
            builder.addDouble("SetpointRpm", () -> setpointRevolutionsPerMinute, val -> setpointRevolutionsPerMinute = val);
        });
    }

    public void setEnabled(boolean enabled) {
        if (enabled == isEnabled || !DriverStation.isTeleop()) {
            return;
        }
        isEnabled = enabled;
    }

    @Override
    public void execute() {
        if (isEnabled) {
            wheel.setTargetRotationsPerMinute(setpointRevolutionsPerMinute);
        } else {
            wheel.setTargetOutput(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        wheel.setTargetOutput(0.0);
    }
}
