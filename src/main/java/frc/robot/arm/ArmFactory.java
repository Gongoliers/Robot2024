package frc.robot.arm;

import frc.lib.controller.PositionControllerIO;
import frc.lib.controller.PositionControllerIOSim;
import frc.lib.controller.PositionControllerIOTalonFX2;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Subsystem;
import frc.robot.arm.ArmConstants.ShoulderConstants;

/** Helper class for creating hardware for the arm subsystem. */
public class ArmFactory {

  /**
   * Creates a shoulder motor.
   *
   * @return a shoulder motor.
   */
  public static PositionControllerIO createShoulder() {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.ARM)) {
      return new PositionControllerIOTalonFX2(
          ShoulderConstants.LEADER_CAN,
          ShoulderConstants.FOLLOWER_CAN,
          ShoulderConstants.ENCODER_CAN,
          ShoulderConstants.PIDF,
          false,
          true);
    }

    return new PositionControllerIOSim();
  }
}
