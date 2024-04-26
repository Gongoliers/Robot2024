package frc.robot.arm;

import frc.lib.CAN;
import frc.lib.config.MechanismConfig;
import frc.lib.controller.PositionControllerIO;
import frc.lib.controller.PositionControllerIOSim;
import frc.lib.controller.PositionControllerIOTalonFX2;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Subsystem;

/** Factory for creating arm subsystem hardware. */
public class ArmFactory {

  /**
   * Creates the shoulder controller.
   *
   * @return the shoulder controller.
   */
  public static PositionControllerIO createShoulder(MechanismConfig config) {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.ARM)) {
      return new PositionControllerIOTalonFX2(
          new CAN(48), new CAN(46), new CAN(52), config, false, true);
    }

    return new PositionControllerIOSim();
  }
}
