package frc.lib.controller;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.Telemetry;

/** Velocity controller interface. */
public interface VelocityControllerIO {

  /** Velocity controller values. */
  public static class VelocityControllerIOValues {
    /** Position in rotations. */
    public double positionRotations = 0.0;

    /** Velocity in rotations per second. */
    public double velocityRotationsPerSecond = 0.0;

    /** Acceleration in rotations per second per second. */
    public double accelerationRotationsPerSecondPerSecond = 0.0;

    /** Motor voltage in volts. */
    public double motorVolts = 0.0;

    /** Motor current in amps. */
    public double motorAmps = 0.0;
  }

  /**
   * Adds velocity controller values to Shuffleboard.
   *
   * @param tab
   * @param name
   * @param values
   */
  public static void addToShuffleboard(
      ShuffleboardTab tab, String name, VelocityControllerIOValues values) {
    ShuffleboardLayout velocityController = Telemetry.addColumn(tab, name);

    velocityController.addDouble("Position (rot)", () -> values.positionRotations);
    velocityController.addDouble("Velocity (rps)", () -> values.velocityRotationsPerSecond);
    velocityController.addDouble(
        "Acceleration (rpsps)", () -> values.accelerationRotationsPerSecondPerSecond);
    velocityController.addDouble("Voltage (V)", () -> values.motorVolts);
    velocityController.addDouble("Current (A)", () -> values.motorAmps);
  }

  /**
   * Configures the velocity controller.
   *
   * @param constants
   */
  public void configure();

  /**
   * Updates the velocity controller's values.
   *
   * @param values
   */
  public void update(VelocityControllerIOValues values);

  /**
   * Sets the velocity controller position.
   *
   * @param positionRotations
   */
  public void setPosition(double positionRotations);

  /**
   * Sets the velocity setpoint.
   *
   * @param velocityRotationsPerSecond
   */
  public void setSetpoint(double velocityRotationsPerSecond);
}
