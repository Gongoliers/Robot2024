package frc.robot.intake;

import frc.lib.CAN;
import frc.lib.PIDFConstants;
import frc.lib.controller.VelocityControllerIO;
import frc.lib.controller.VelocityControllerIOTalonFXPIDF;

/** Helper class for creating hardware for the intake subsystem. */
public class IntakeFactory {

  public static VelocityControllerIO createFrontRoller() {
    PIDFConstants pidf = new PIDFConstants();

    pidf.kS = 0.13;
    pidf.kV = 0.1683;

    return new VelocityControllerIOTalonFXPIDF(new CAN(50), pidf);
  }

  public static VelocityControllerIO createBackRoller() {
    PIDFConstants pidf = new PIDFConstants();

    pidf.kS = 0.13;
    pidf.kV = 0.1759;

    return new VelocityControllerIOTalonFXPIDF(new CAN(40), pidf);
  }
}
