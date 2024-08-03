package frc.robot.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.SmarterDashboard;

public class ArmTuningCommand extends Command {

    public static final double DEFAULT_TARGET_POSITION = Units.degreesToRadians(40.0);
    public static final double DT_SECS = 0.02;

    private final ArmSubsystem arm;
    private double setpointDegrees;
    private double setpointVelocity;
    private double targetDegrees;
    private boolean enabled;

    public ArmTuningCommand(ArmSubsystem arm) {

        this.arm = arm;
        this.setpointDegrees = DEFAULT_TARGET_POSITION;
        this.targetDegrees = 0.0;

        addRequirements(arm);

        SmarterDashboard.putData("ArmTuningCommand", builder -> {
            builder.addBoolean("Enabled?", () -> enabled, this::setEnabled);
            builder.addDouble("kP", arm::getP, arm::setP);
            builder.addDouble("kG", arm::getG, arm::setG);
            builder.addDouble("kV", arm::getV, arm::setV);
            builder.addDouble("SetpointDegrees", () -> setpointDegrees, this::setSetpointDegrees);
            builder.addDouble("SetpointVelocity", () -> setpointVelocity, val -> setpointVelocity = val);
            builder.addDouble("TargetDegrees", () -> targetDegrees);
        });
    }

    private void setSetpointDegrees(double newTarget) {
        setpointDegrees = newTarget;
        targetDegrees = setpointDegrees;
    }

    private void setEnabled(boolean shouldEnable) {
        if (shouldEnable == enabled || !DriverStation.isTeleop()) {
            return;
        }
        if (shouldEnable) {
            targetDegrees = setpointDegrees;
        }
        enabled = shouldEnable;
    }

    @Override
    public void execute() {
        if (enabled) {
            arm.setTargetDegreesAndVelocity(targetDegrees, setpointVelocity);
            if (setpointVelocity != 0.0) {
                targetDegrees += setpointVelocity * DT_SECS;
            }
        } else {
            arm.setTargetOutput(0.0);
        }
    }
}
