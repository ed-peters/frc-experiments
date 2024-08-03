package frc.robot.wheel;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SmarterDashboard;

/**
 * Simple subsystem that controls a spinning wheel. Allows setting the a target output
 * (-1.0 - 1.0) or a target speed (in rotations per minute).
 */
public class WheelSubsystem extends SubsystemBase {

    // Set these based on the hardware configuration of the robot
    public static final double GEAR_RATIO = 1.0;
    public static final double WHEEL_CIRCUMFERENCE = 1.0;
    public static final boolean INVERTED = false;

    // Safety limits
    public static final double MAX_RPM = 5600.0;
    public static final int MAX_AMPS = 40;
    public static final double RAMP_RATE = 0.05;

    // Tuning parameters
    public static final double KP = 1.0;
    public static final double KV = 1.0;

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkPIDController pid;
    private SimpleMotorFeedforward ff;
    private double currentOutputAmps;
    private double currentRotationsPerMinute;
    private double currentFeetPerSecond;
    private double lastFeedforward;
    private double targetOutput;
    private double targetRotationsPerMinute;
    private boolean brakeEnabled;

    public WheelSubsystem(int canId) {

        motor = new CANSparkMax(canId, CANSparkLowLevel.MotorType.kBrushless);
        motor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        motor.setSmartCurrentLimit(MAX_AMPS);
        motor.setInverted(INVERTED);
        motor.setOpenLoopRampRate(RAMP_RATE);
        motor.setClosedLoopRampRate(RAMP_RATE);
        motor.setIdleMode(CANSparkBase.IdleMode.kCoast);

        brakeEnabled = false;

        encoder = motor.getEncoder();

        // By applying the gear ratio here, the velocity returned by the encoder
        // will be the velocity of the attached wheel (not the motor), in the usual
        // REV units which are rotations per minute
        encoder.setVelocityConversionFactor(GEAR_RATIO);

        pid = motor.getPIDController();
        pid.setP(KP);

        ff = new SimpleMotorFeedforward(0.0, KV);

        targetOutput = 0.0;
        targetRotationsPerMinute = Double.NaN;
        lastFeedforward = 0.0;

        SmarterDashboard.putData("WheelSubsystem", builder -> {
            builder.addBoolean("BrakeEnabled?", () -> brakeEnabled, this::setBrakeEnabled);
            builder.addDouble("CurrentAmps", () -> currentOutputAmps);
            builder.addDouble("CurrentFps", () -> currentFeetPerSecond);
            builder.addDouble("CurrentRpm", () -> currentRotationsPerMinute);
            builder.addDouble("LastFeedforward", () -> lastFeedforward);
            builder.addDouble("TargetOutput", () -> targetOutput);
            builder.addDouble("TargetRpm", () -> targetRotationsPerMinute);
        });
    }

    public double getP() {
        return pid.getP();
    }

    public void setP(double kP) {
        pid.setP(kP);
    }

    public double getV() {
        return ff.kv;
    }

    public void setV(double kV) {
        ff = new SimpleMotorFeedforward(0.0, kV);
    }

    public void setTargetOutput(double output) {
        targetOutput = MathUtil.clamp(output, -1.0, 1.0);
        targetRotationsPerMinute = Double.NaN;
    }

    public void setTargetRotationsPerMinute(double rpm) {
        targetOutput = Double.NaN;
        targetRotationsPerMinute = MathUtil.clamp(rpm, -MAX_RPM, MAX_RPM);
    }

    public void setBrakeEnabled(boolean enabled) {
        if (brakeEnabled != enabled) {
            motor.setIdleMode(enabled ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
            brakeEnabled = enabled;
        }
    }

    @Override
    public void periodic() {

        currentOutputAmps = motor.getOutputCurrent();
        currentRotationsPerMinute = encoder.getVelocity();
        currentFeetPerSecond = currentRotationsPerMinute * WHEEL_CIRCUMFERENCE / 60.0;

        if (Double.isFinite(targetRotationsPerMinute)) {
            lastFeedforward = ff.calculate(targetRotationsPerMinute);
            pid.setReference(targetRotationsPerMinute, CANSparkBase.ControlType.kVelocity, 0, lastFeedforward);
        } else {
            motor.set(targetOutput);
        }
    }

    public Command rpmCommand(double rpm) {
        return run(() -> this.setTargetRotationsPerMinute(rpm));
    }

    public Command stopCommand() {
        return run(() -> this.setTargetOutput(0.0));
    }
}
