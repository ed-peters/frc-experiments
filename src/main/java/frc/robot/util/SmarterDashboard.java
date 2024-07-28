package frc.robot.util;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.*;

/**
 * Wrapper for the {@link SmartDashboard} methods that saves a little typing and adds
 * some unit conversions
 */
public class SmarterDashboard {

    public static void putData(String name, Consumer<DashboardBuilder> consumer) {
        SmartDashboard.putData(name, builder -> consumer.accept(new DashboardBuilder(builder)));
    }

    public static class DashboardBuilder {

        private final SendableBuilder builder;

        public DashboardBuilder(SendableBuilder builder) {
            this.builder = builder;
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
    }
}
