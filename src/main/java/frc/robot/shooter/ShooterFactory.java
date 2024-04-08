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
   * Creates a serializer motor.
   *
   * @return a serializer motor.
   */
  public static VelocityControllerIO createSerializerMotor() {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.SHOOTER)) {
      return new VelocityControllerIOTalonFXPIDF(SerializerConstants.CAN, SerializerConstants.PIDF);
    }

    return new VelocityControllerIOSim();
  }

  /**
   * Creates a flywheel motor.
   *
   * @return a flywheel motor.
   */
  public static VelocityControllerIO createFlywheelMotor() {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.SHOOTER)) {
      return new VelocityControllerIOTalonFXPIDF(FlywheelConstants.CAN, FlywheelConstants.PIDF);
    }

    return new VelocityControllerIOSim();
  }
}
