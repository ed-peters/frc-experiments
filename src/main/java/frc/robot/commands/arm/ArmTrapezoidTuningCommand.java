package frc.robot.commands.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.util.SmarterDashboard;

public class ArmTrapezoidTuningCommand extends Command {

    public static final double DEFAULT_MAX_VELOCITY = 30.0;
    public static final double DEFAULT_MAX_ACCELERATION = 30.0;
    public static final double DEFAULT_SETPOINT = 30.0;

    private final ArmSubsystem arm;
    private final State currentState;
    private final State targetState;
    private State nextState;
    private double maxVelocity;
    private double maxAcceleration;
    private TrapezoidProfile profile;
    private boolean isEnabled;

    public ArmTrapezoidTuningCommand(ArmSubsystem arm) {

        this.arm = arm;
        this.currentState = new State();
        this.targetState = new State();
        this.isEnabled = false;

        setTargetDegrees(DEFAULT_SETPOINT);
        updateProfile(DEFAULT_MAX_VELOCITY, DEFAULT_MAX_ACCELERATION);

        addRequirements(arm);

        SmarterDashboard.putData("ArmTrapezoidTuningCommand", builder -> {
            builder.addBoolean("Enabled?", () -> isEnabled, this::setEnabled);
            builder.addDouble("MaxVelocity", () -> maxVelocity, this::setMaxVelocity);
            builder.addDouble("MaxAcceleration", () -> maxAcceleration, this::setMaxAcceleration);
            builder.addDouble("SetpointDegrees", () -> targetState.position, this::setTargetDegrees);
            builder.addDouble("NextDegrees", () -> nextState == null ? Double.NaN : nextState.position);
            builder.addDouble("NextVelocity", () -> nextState == null ? Double.NaN : nextState.velocity);
        });
    }

    public void setEnabled(boolean enabled) {
        if (enabled == isEnabled || !DriverStation.isTeleop()) {
            return;
        }
        isEnabled = enabled;
    }

    private void setMaxVelocity(double newVelocity) {
        if (newVelocity != maxVelocity) {
            updateProfile(newVelocity, maxAcceleration);
        }
    }

    private void setMaxAcceleration(double newAcceleration) {
        if (newAcceleration != maxAcceleration) {
            updateProfile(maxVelocity, newAcceleration);
        }
    }

    private void updateProfile(double newVelocity, double newAcceleration) {
        maxVelocity = newVelocity;
        maxAcceleration = newAcceleration;
        profile = new TrapezoidProfile(new Constraints(maxVelocity, maxAcceleration));
    }

    private void setTargetDegrees(double degrees) {
        targetState.position = degrees;
        targetState.velocity = 0.0;
    }

    @Override
    public void execute() {
        currentState.position = arm.getCurrentDegrees();
        currentState.velocity = arm.getCurrentVelocity();
        if (isEnabled) {
            nextState = profile.calculate(0.02, currentState, targetState);
            arm.setTargetDegreesAndVelocity(nextState.position, nextState.velocity);
        } else {
            nextState = null;
            arm.setTargetOutput(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        nextState = null;
        arm.setTargetOutput(0.0);
    }
}
