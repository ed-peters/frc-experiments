package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SmarterDashboard;

public class ArmSubsystem extends SubsystemBase {

    // Set these based on the hardware configuration of the robot
    public static final boolean INVERT_LEADER = false;
    public static final boolean INVERT_FOLLOWER = true;

    public static final double ENCODER_OFFSET = 0;
    public static final double DEGREES_PER_ROTATION = 360.0 * (1.0 / (75.0 / (24.0 / 64.0)));
    public static final double ERROR_TOLERANCE = 2.0;

    // Safety limits
    public static final double MIN_ANGLE = 0.0;
    public static final double MAX_ANGLE = 105.0;
    public static final double RAMP_RATE = 0.25;
    public static final int MAX_AMPS = 30;

    // Tuning parameters
    public static final double kP = 0.018;
    public static final double kG = 0.42;
    public static final double kV = 0.0;
    private final CANSparkMax leadMotor;
    private final CANSparkMax followMotor;
    private final SparkPIDController leadPid;
    private final RelativeEncoder leadEncoder;
    private final RelativeEncoder followEncoder;
    private final DutyCycleEncoder leadThroughboreEncoder;
    private ArmFeedforward feedforward;
    private double currentAmps;
    private double currentDegrees;
    private double currentVelocity;
    private double currentRotations;
    private boolean brakeEnabled;
    private boolean initialized = false;
    private double targetDegrees;
    private double targetVelocity;
    private double targetOutput;
    private double lastFeedforward;
    private String isLimited;

    public ArmSubsystem(int leadMotorId, int followMotorId) {

        leadMotor = new CANSparkMax(leadMotorId, CANSparkLowLevel.MotorType.kBrushless);
        leadMotor.setInverted(INVERT_LEADER);
        leadMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        leadMotor.setOpenLoopRampRate(RAMP_RATE);
        leadMotor.setSmartCurrentLimit(MAX_AMPS);

        followMotor = new CANSparkMax(followMotorId, CANSparkLowLevel.MotorType.kBrushless);
        followMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        followMotor.setOpenLoopRampRate(RAMP_RATE);
        followMotor.setSmartCurrentLimit(MAX_AMPS);
        followMotor.follow(leadMotor, INVERT_FOLLOWER);

        leadEncoder = leadMotor.getEncoder();
        followEncoder = followMotor.getEncoder();

        leadThroughboreEncoder = new DutyCycleEncoder(1);

        isLimited = "";

        leadPid = leadMotor.getPIDController();
        leadPid.setP(kP);
        leadPid.setI(0.0);
        leadPid.setD(0.0);

        feedforward = new ArmFeedforward(0.0, kG, kV);
        setTargetOutput(0.0);
        setBrakeEnabled(true);

        SmarterDashboard.putData("ArmSubsystem", builder -> {
            builder.addDouble("AbsolutePosition", leadThroughboreEncoder::getAbsolutePosition);
            builder.addString("AtLimit?", () -> isLimited);
            builder.addBoolean("BrakeEnabled?", () -> brakeEnabled, this::setBrakeEnabled);
            builder.addDouble("CurrentAmps", () -> currentAmps);
            builder.addDouble("CurrentDegrees", () -> currentDegrees);
            builder.addDouble("CurrentVelocity", () -> currentVelocity);
            builder.addDouble("CurrentRotations", () -> currentRotations);
            builder.addBoolean("Initialized??", () -> initialized);
            builder.addDouble("LastFeedforward", () -> lastFeedforward);
            builder.addDouble("TargetDegrees", () -> targetDegrees);
            builder.addDouble("TargetVelocity", () -> targetVelocity);
            builder.addDouble("TargetOutput", () -> targetOutput);
        });
    }

    private double getAbsolutePosition() {
        double initPos = leadThroughboreEncoder.getAbsolutePosition() - ENCODER_OFFSET;
        if (initPos > 0.5) {
            initPos = initPos - 1.0;
        } else if (initPos <= -0.5) {
            initPos = initPos + 1.0;
        }
        return initPos;
    }

    public double getCurrentDegrees() {
        return currentDegrees;
    }

    public double getCurrentVelocity() {
        return currentVelocity;
    }

    public double getP() {
        return leadPid.getP();
    }

    public void setP(double kP) {
        leadPid.setP(kP);
    }

    public double getG() {
        return feedforward.kg;
    }

    public void setG(double kG) {
        feedforward = new ArmFeedforward(0.0, kG, feedforward.kv);
    }

    public double getV() {
        return feedforward.kv;
    }

    public void setV(double kV) {
        feedforward = new ArmFeedforward(0.0, feedforward.kg, kV);
    }

    public void setBrakeEnabled(boolean newValue) {
        brakeEnabled = newValue;
        leadMotor.setIdleMode(newValue ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
        if (followMotor != null) {
            followMotor.setIdleMode(newValue ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
        }
    }

    public void setTargetDegrees(double degrees) {
        setTargetDegreesAndVelocity(degrees, 0.0);
    }

    public void setTargetDegreesAndVelocity(double degrees, double velocity) {
        targetDegrees = MathUtil.clamp(degrees, -MIN_ANGLE, MAX_ANGLE);
        targetVelocity = velocity;
        targetOutput = Double.NaN;
    }

    public void setTargetOutput(double output) {
        targetDegrees = Double.NaN;
        targetVelocity = Double.NaN;
        targetOutput = MathUtil.clamp(output, -1.0, 1.0);
    }

    public boolean isAtSetpoint() {
        if (Double.isFinite(targetDegrees)) {
            double delta = Math.abs(currentDegrees - targetDegrees);
            return delta < ERROR_TOLERANCE;
        }
        return false;
    }

    @Override
    public void periodic() {

        if (!initialized) {
            leadEncoder.setPosition(getAbsolutePosition());
            followEncoder.setPosition(getAbsolutePosition());
            initialized = true;
        }

        currentAmps = leadMotor.getOutputCurrent();
        currentRotations = leadEncoder.getPosition();
        currentVelocity = leadEncoder.getVelocity() * DEGREES_PER_ROTATION;
        currentDegrees = currentRotations * DEGREES_PER_ROTATION;
        isLimited = "";

        // In open-loop mode, we have to make sure we stop the motors when
        // we've exceeded the angle limits
        if (Double.isFinite(targetOutput)) {
            if (currentDegrees < MIN_ANGLE && targetOutput < 0.0) {
                targetOutput = 0.0;
                isLimited = "MIN";
            }
            else if (currentDegrees > MAX_ANGLE && targetOutput > 0.0) {
                targetOutput = 0.0;
                isLimited = "MAX";
            }
            leadMotor.set(targetOutput);
        }

        // In open-loop mode, we have to calculate feedforward (notice that
        // we have to use radians, because that's how WPILib does it) and
        // then apply the target position to the motors.
        else {
            lastFeedforward = feedforward.calculate(Units.degreesToRadians(currentDegrees), targetVelocity);
            leadPid.setReference(
                    targetDegrees / DEGREES_PER_ROTATION,
                    CANSparkBase.ControlType.kPosition, 0, lastFeedforward);
        }
    }
}
