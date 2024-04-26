package frc.robot.intake;

import frc.lib.CAN;
import frc.lib.config.MechanismConfig;
import frc.lib.controller.VelocityControllerIO;
import frc.lib.controller.VelocityControllerIOSim;
import frc.lib.controller.VelocityControllerIOTalonFXPIDF;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Subsystem;

/** Helper class for creating hardware for the intake subsystem. */
public class IntakeFactory {

  public static VelocityControllerIO createFrontRoller(MechanismConfig config) {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.INTAKE)) {
      return new VelocityControllerIOTalonFXPIDF(new CAN(50), config);
    }

    return new VelocityControllerIOSim();
  }

  public static VelocityControllerIO createBackRoller(MechanismConfig config) {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.INTAKE)) {
      return new VelocityControllerIOTalonFXPIDF(new CAN(40), config);
    }

    return new VelocityControllerIOSim();
  }
}
