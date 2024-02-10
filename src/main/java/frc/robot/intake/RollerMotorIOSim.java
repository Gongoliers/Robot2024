package frc.robot.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.RobotConstants;
import frc.robot.intake.IntakeConstants.RollerMotorConstants;

/** Simulated roller motor. */
public class RollerMotorIOSim implements RollerMotorIO {

  /** Motor used to drive the roller. */
  private final DCMotor motorSim = DCMotor.getNeo550(1);

  /** Rollers driven by the motor. */
  private final FlywheelSim flywheelSim =
      new FlywheelSim(motorSim, RollerMotorConstants.GEARING, RollerMotorConstants.MOI);

  @Override
  public void configure() {}

  @Override
  public void update(RollerMotorIOValues values) {
    flywheelSim.update(RobotConstants.PERIODIC_DURATION);

    values.angularVelocityRotationsPerSecond = flywheelSim.getAngularVelocityRPM() / 60.0;
    values.currentAmps = flywheelSim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    flywheelSim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    setVoltage(0);
  }
}
