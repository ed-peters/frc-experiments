package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;
import java.util.function.*;

/**
 * Wrapper for the {@link SmartDashboard} methods that saves a little typing and adds
 * some unit conversions
 */
public class SmarterDashboard {

    public static void putChooser(String name, SendableChooser<?> chooser) {
        SmartDashboard.putData(name, chooser);
    }

    public static void putData(String name, Consumer<DashboardBuilder> consumer) {
        SmartDashboard.putData(name, builder -> consumer.accept(new DashboardBuilder(builder)));
    }

    public static class DashboardBuilder {

        private final SendableBuilder builder;

        public DashboardBuilder(SendableBuilder builder) {
            this.builder = builder;
        }

        public void addDoubleArray(String name, Supplier<double[]> getter) {
            builder.addDoubleArrayProperty(name, getter, null);
        }

        public void addDouble(String name, DoubleSupplier getter) {
            addDouble(name, getter, null);
        }

        public void addDouble(String name, DoubleSupplier getter, DoubleConsumer setter) {
            builder.addDoubleProperty(name, getter, setter);
        }

        public void displayAsFeet(String name, DoubleSupplier getter) {
            addDouble(name, getter, null);
        }

        public void displayAsFeet(String name, DoubleSupplier getter, DoubleConsumer setter) {
            DoubleSupplier wrappedGetter = () -> Units.metersToFeet(getter.getAsDouble());
            DoubleConsumer wrappedSetter = null;
            if (setter != null) {
                wrappedSetter = feet -> setter.accept(Units.feetToMeters(feet));
            }
            addDouble(name, wrappedGetter, wrappedSetter);
        }

        public void displayAsDegrees(String name, DoubleSupplier getter) {
            addDouble(name, getter, null);
        }

        public void displayAsDegrees(String name, DoubleSupplier getter, DoubleConsumer setter) {
            DoubleSupplier wrappedGetter = () -> Units.radiansToDegrees(getter.getAsDouble());
            DoubleConsumer wrappedSetter = null;
            if (setter != null) {
                wrappedSetter = deg -> setter.accept(Units.degreesToRadians(deg));
            }
            addDouble(name, wrappedGetter, wrappedSetter);
        }

        public void displayAsRotations(String name, DoubleSupplier getter) {
            addDouble(name, getter, null);
        }

        public void displayAsRotations(String name, DoubleSupplier getter, DoubleConsumer setter) {
            DoubleSupplier wrappedGetter = () -> Units.radiansToRotations(getter.getAsDouble());
            DoubleConsumer wrappedSetter = null;
            if (setter != null) {
                wrappedSetter = rot -> setter.accept(Units.rotationsToRadians(rot));
            }
            addDouble(name, wrappedGetter, wrappedSetter);
        }

        public void addString(String name, Supplier<String> getter) {
            builder.addStringProperty(name, getter, null);
        }

        public void addBoolean(String name, BooleanSupplier getter) {
            addBoolean(name, getter, null);
        }

        public void addBoolean(String name, BooleanSupplier getter, BooleanConsumer setter) {
            builder.addBooleanProperty(name, getter, setter);
        }

        public void addSpeeds(String name, Supplier<ChassisSpeeds> supplier) {
            final double [] data = new double[3];
            addDoubleArray(name, () -> {
                ChassisSpeeds speeds = supplier.get();
                if (speeds == null) {
                    Arrays.fill(data, 0.0);
                } else {
                    data[0] = Units.metersToFeet(speeds.vxMetersPerSecond);
                    data[1] = Units.metersToFeet(speeds.vyMetersPerSecond);
                    data[2] = Units.radiansToDegrees(speeds.omegaRadiansPerSecond);
                }
                return data;
            });
        }

        public void addPose(String name, Supplier<Pose2d> supplier) {
            final double [] data = new double[3];
            addDoubleArray(name, () -> {
                Pose2d pose = supplier.get();
                if (pose == null) {
                    Arrays.fill(data, 0.0);
                } else {
                    data[0] = pose.getX();
                    data[1] = pose.getY();
                    data[2] = pose.getRotation().getDegrees();
                }
                return data;
            });
        }
    }
}
