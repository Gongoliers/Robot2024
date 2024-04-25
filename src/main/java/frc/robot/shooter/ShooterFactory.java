package frc.robot.shooter;

import frc.lib.controller.VelocityControllerIO;
import frc.lib.controller.VelocityControllerIOSim;
import frc.lib.controller.VelocityControllerIOTalonFXPIDF;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Subsystem;
import frc.robot.shooter.ShooterConstants.FlywheelConstants;
import frc.robot.shooter.ShooterConstants.SerializerConstants;

/** Helper class for creating hardware for the shooter subsystem. */
public class ShooterFactory {

  /**
   * Creates a serializer.
   *
   * @return a serializer.
   */
  public static VelocityControllerIO createSerializer() {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.SHOOTER)) {
      return new VelocityControllerIOTalonFXPIDF(
          SerializerConstants.CAN, SerializerConstants.PIDF_CONTROLLER_CONSTANTS);
    }

    return new VelocityControllerIOSim();
  }

  /**
   * Creates a flywheel.
   *
   * @return a flywheel.
   */
  public static VelocityControllerIO createFlywheel() {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.SHOOTER)) {
      return new VelocityControllerIOTalonFXPIDF(
          FlywheelConstants.CAN, FlywheelConstants.CONFIG);
    }

    return new VelocityControllerIOSim();
  }
}
