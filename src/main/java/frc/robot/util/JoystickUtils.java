package frc.robot.util;

import static edu.wpi.first.wpilibj.XboxController.Axis;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.function.DoubleSupplier;

public class JoystickUtils {

  public static DoubleSupplier getAxis(CommandXboxController controller, Axis axis) {
    final XboxController xboxController = controller.getHID();
    return () -> -xboxController.getRawAxis(axis.value);
  }
}
