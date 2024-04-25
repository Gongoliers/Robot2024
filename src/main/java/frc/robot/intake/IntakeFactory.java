package frc.robot.intake;

import frc.lib.controller.VelocityControllerIO;
import frc.lib.controller.VelocityControllerIOSim;
import frc.lib.controller.VelocityControllerIOTalonFXPIDF;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Subsystem;
import frc.robot.intake.IntakeConstants.BackRollerConstants;
import frc.robot.intake.IntakeConstants.FrontRollerConstants;

/** Helper class for creating hardware for the intake subsystem. */
public class IntakeFactory {

  public static VelocityControllerIO createFrontRoller() {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.INTAKE)) {
      return new VelocityControllerIOTalonFXPIDF(
          FrontRollerConstants.CAN, FrontRollerConstants.PIDF_CONTROLLER_CONSTANTS);
    }

    return new VelocityControllerIOSim();
  }

  public static VelocityControllerIO createBackRoller() {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.INTAKE)) {
      return new VelocityControllerIOTalonFXPIDF(
          BackRollerConstants.CAN, BackRollerConstants.PIDF_CONTROLLER_CONSTANTS);
    }

    return new VelocityControllerIOSim();
  }
}
