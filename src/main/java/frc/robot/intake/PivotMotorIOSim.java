package frc.robot.intake;

import edu.wpi.first.wpilibj.DriverStation;

/** Simulated pivot motor. */
public class PivotMotorIOSim implements PivotMotorIO {

  private double positionRotations;
  private double velocityRotationsPerSecond;

  @Override
  public void configure() {}

  @Override
  public void update(PivotMotorIOValues values) {
    values.positionRotations = this.positionRotations;
    ;
    values.velocityRotationsPerSecond = this.velocityRotationsPerSecond;
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
}
