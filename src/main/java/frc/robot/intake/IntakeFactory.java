package frc.robot.intake;

import frc.lib.CAN;
import frc.lib.config.MechanismConfig;
import frc.lib.controller.VelocityControllerIO;
import frc.lib.controller.VelocityControllerIOSim;
import frc.lib.controller.VelocityControllerIOTalonFXPIDF;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Subsystem;

/** Factory for creating intake subsystem hardware. */
public class IntakeFactory {

  /**
   * Creates the front roller controller.
   *
   * @param config the front roller controller config.
   * @return the front roller controller.
   */
  public static VelocityControllerIO createFrontRoller(MechanismConfig config) {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.INTAKE)) {
      return new VelocityControllerIOTalonFXPIDF(new CAN(50), config);
    }

    return new VelocityControllerIOSim();
  }

  /**
   * Creates the back roller controller.
   *
   * @param config the back roller controller config.
   * @return the back roller controller.
   */
  public static VelocityControllerIO createBackRoller(MechanismConfig config) {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.INTAKE)) {
      return new VelocityControllerIOTalonFXPIDF(new CAN(40), config);
    }

    return new VelocityControllerIOSim();
  }
}
