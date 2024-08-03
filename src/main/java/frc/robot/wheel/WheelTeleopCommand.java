package frc.robot.wheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.SmarterDashboard;

import java.util.function.DoubleSupplier;

/**
 * Teleop control for a {@link WheelSubsystem}
 */
public class WheelTeleopCommand extends Command {

    public static final double DEADBAND = 0.1;

    private final WheelSubsystem wheel;
    private final DoubleSupplier percent;
    private double lastOutput;

    public WheelTeleopCommand(WheelSubsystem wheel, DoubleSupplier percent) {

        this.wheel = wheel;
        this.percent = percent;
        this.lastOutput = 0.0;

        addRequirements(wheel);

        SmarterDashboard.putData("WheelTeleopCommand", builder -> {
            builder.addDouble("LastOutput", () -> lastOutput);
        });
    }

    @Override
    public void execute() {
        double p = percent.getAsDouble();
        p = MathUtil.applyDeadband(p, DEADBAND);
        p = MathUtil.clamp(p, -1.0, 1.0);
        setTargetOutput(p);
    }

    @Override
    public void end(boolean interrupted) {
        setTargetOutput(0.0);
    }

    private void setTargetOutput(double output) {
        lastOutput = output;
        wheel.setTargetOutput(lastOutput);
    }
}
