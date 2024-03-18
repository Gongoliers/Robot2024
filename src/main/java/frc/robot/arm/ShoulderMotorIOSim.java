package frc.robot.arm;

import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.AccelerationCalculator;

/** Simulated shoulder motor. */
public class ShoulderMotorIOSim implements ShoulderMotorIO {

  private double positionRotations;
  private double velocityRotationsPerSecond;
  private double inputVoltage;

  private final AccelerationCalculator accelerationCalculator;

  /** Creates a new simulated shoulder motor. */
  public ShoulderMotorIOSim() {
    accelerationCalculator = new AccelerationCalculator();
  }

  @Override
  public void configure() {}

  @Override
  public void update(ShoulderMotorIOValues values) {
    values.positionRotations = this.positionRotations;
    values.velocityRotationsPerSecond = this.velocityRotationsPerSecond;
    values.accelerationRotationsPerSecondPerSecond =
        accelerationCalculator.calculate(velocityRotationsPerSecond);

    values.inputVoltage = inputVoltage;
  }

  @Override
  public void setPosition(double positionRotations) {
    this.positionRotations = positionRotations;
  }

  @Override
  public void setSetpoint(double positionRotations, double velocityRotationsPerSecond) {
    if (DriverStation.isEnabled()) {
      this.positionRotations = positionRotations;
      this.velocityRotationsPerSecond = velocityRotationsPerSecond;
    }
  }

  @Override
  public void setVoltage(double volts) {
    this.inputVoltage = volts;
  }
}
