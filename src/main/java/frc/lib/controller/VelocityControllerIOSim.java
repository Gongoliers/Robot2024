package frc.lib.controller;

import frc.robot.RobotConstants;

/** Simulated velocity controller. */
public class VelocityControllerIOSim implements VelocityControllerIO {

  private double positionRotations = 0.0;

  private double velocityRotationsPerSecond = 0.0;

  @Override
  public void configure() {}

  @Override
  public void update(VelocityControllerIOValues values) {
    positionRotations += velocityRotationsPerSecond * RobotConstants.PERIODIC_DURATION;

    values.positionRotations = positionRotations;
    values.velocityRotationsPerSecond = velocityRotationsPerSecond;
  }

  @Override
  public void setPosition(double positionRotations) {
    this.positionRotations = positionRotations;
  }

  @Override
  public void setSetpoint(double velocityRotationsPerSecond) {
    this.velocityRotationsPerSecond = velocityRotationsPerSecond;
  }
}
