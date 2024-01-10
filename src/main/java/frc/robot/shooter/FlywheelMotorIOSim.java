package frc.robot.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.RobotConstants;
import frc.robot.shooter.ShooterConstants.FlywheelConstants;

/** Simulated flywheel motor. */
public class FlywheelMotorIOSim implements FlywheelMotorIO {

  /** Represents the motor used the drive the flywheel. */
  private final DCMotor motorSim = DCMotor.getNeo550(1);

  /** Represents the flywheel driven by the motor. */
  private final FlywheelSim flywheelSim =
      new FlywheelSim(motorSim, FlywheelConstants.GEARING, FlywheelConstants.MOI);

  @Override
  public void configure() {}

  @Override
  public void update(FlywheelMotorIOValues values) {
    flywheelSim.update(RobotConstants.TICK_PERIOD);

    values.angularVelocityRotationsPerSecond = flywheelSim.getAngularVelocityRPM() / 60.0;
    values.currentAmps = flywheelSim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    flywheelSim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    flywheelSim.setInputVoltage(0);
  }
}
