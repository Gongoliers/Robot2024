package frc.robot.shooter;

import frc.lib.CAN;
import frc.lib.config.MechanismConfig;
import frc.lib.controller.VelocityControllerIO;
import frc.lib.controller.VelocityControllerIOSim;
import frc.lib.controller.VelocityControllerIOTalonFXPIDF;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Subsystem;

/** Factory for creating shooter subsystem hardware. */
public class ShooterFactory {

  /**
   * Creates the flywheel controller.
   *
   * @param config the flywheel controller config.
   * @return the flywheel controller.
   */
  public static VelocityControllerIO createFlywheel(MechanismConfig config) {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.SHOOTER)) {
      return new VelocityControllerIOTalonFXPIDF(new CAN(44), config);
    }

    return new VelocityControllerIOSim();
  }

  /**
   * Creates the serializer controller.
   *
   * @param config the serializer controller config.
   * @return the serializer controller.
   */
  public static VelocityControllerIO createSerializer(MechanismConfig config) {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.SHOOTER)) {
      return new VelocityControllerIOTalonFXPIDF(new CAN(42), config);
    }

    return new VelocityControllerIOSim();
  }
}
