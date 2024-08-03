package frc.robot.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.SmarterDashboard;

import java.util.function.DoubleSupplier;

/**
 * Teleop control for the arm. Moves the arm up and down based on a "percent of max
 * power" input, and holds it still at the last position if the input is zero.
 */
public class ArmTeleopCommand extends Command {

    public static final double DEADBAND = 0.1;
    public static final double MIN_DEGREES_FOR_HOLD = 1.0;

    private final ArmSubsystem arm;
    private final DoubleSupplier percent;
    private double lastOutput;
    private double lastHoldDegrees;

    public ArmTeleopCommand(ArmSubsystem arm, DoubleSupplier percent) {

        this.arm = arm;
        this.percent = percent;
        this.lastOutput = 0.0;
        this.lastHoldDegrees = Double.NaN;

        addRequirements(arm);

        SmarterDashboard.putData("ArmTeleopCommand", builder -> {
            builder.addDouble("LastHoldDegrees", () -> lastHoldDegrees);
            builder.addDouble("LastOutput", () -> lastOutput);
        });
    }

    @Override
    public void initialize() {
        this.lastHoldDegrees = Double.NaN;
    }

    @Override
    public void execute() {

        // Get the input power percentage, apply a deadband and clamp to [-1.0, 1.0]
        lastOutput = percent.getAsDouble();
        lastOutput = MathUtil.applyDeadband(lastOutput, DEADBAND);
        lastOutput = MathUtil.clamp(lastOutput, -1.0, 1.0);

        // If there's no input, and the arm is raised, we want to hold it still at the
        // position it was at when the input first went to 0.
        // TODO debug me
        if (lastOutput == 0.0 && arm.getCurrentDegrees() > MIN_DEGREES_FOR_HOLD) {
            if (Double.isNaN(lastHoldDegrees)) {
                lastHoldDegrees = arm.getCurrentDegrees();
            }
            arm.setTargetDegrees(lastHoldDegrees);
        }

        // Otherwise, we apply a clamp and a deadband, then multiply by the maximum
        // volts allowed by the arm. The result will be a value in the range of
        // (-MAX_VOLTS, MAX_VOLTS).
        else {
            lastHoldDegrees = Double.NaN;
            arm.setTargetOutput(lastOutput);
        }
    }

    @Override
    public void end(boolean interrupted) {

        lastOutput = 0.0;
        lastHoldDegrees = Double.NaN;

        // If this command ends with the arm raised, we want it to hold
        // still in the current position
        if (arm.getCurrentDegrees() > MIN_DEGREES_FOR_HOLD) {
            arm.setTargetDegrees(arm.getCurrentDegrees());
        } else {
            arm.setTargetOutput(0.0);
        }
    }
}
