package frc.robot.swerve;

import frc.robot.RobotConstants;

/** Simulated drive motor. */
public class DriveMotorIOSim implements DriveMotorIO {

  /** Represents the position of the drive motor. */
  private double positionMeters;

  /** Represents the velocity of the drive motor. */
  private double velocityMetersPerSecond;

  @Override
  public void configure() {}

  @Override
  public void update(DriveMotorIOValues values) {
    this.positionMeters += velocityMetersPerSecond * RobotConstants.PERIODIC_DURATION;

    values.positionMeters = positionMeters;
    values.velocityMetersPerSecond = velocityMetersPerSecond;
  }

  @Override
  public void setPosition(double positionMeters) {
    this.positionMeters = positionMeters;
  }

  @Override
  public void runSetpoint(double velocityMetersPerSecond) {
    this.velocityMetersPerSecond = velocityMetersPerSecond;
  }

  @Override
  public void setBrake(boolean brake) {}
}
