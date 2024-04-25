package frc.lib.controller;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.Telemetry;

/** Position controller interface. */
public interface PositionControllerIO {

  /** Position controller values. */
  public static class PositionControllerIOValues {
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
   * Adds position controller values to Shuffleboard.
   *
   * @param tab
   * @param name
   * @param values
   */
  public static void addToShuffleboard(
      ShuffleboardTab tab, String name, PositionControllerIOValues values) {
    ShuffleboardLayout positionController = Telemetry.addColumn(tab, name);

    positionController.addDouble("Position (rot)", () -> values.positionRotations);
    positionController.addDouble("Velocity (rps)", () -> values.velocityRotationsPerSecond);
    positionController.addDouble(
        "Acceleration (rpsps)", () -> values.accelerationRotationsPerSecondPerSecond);
    positionController.addDouble("Voltage (V)", () -> values.motorVolts);
    positionController.addDouble("Current (A)", () -> values.motorAmps);
  }

  /**
   * Configures the position controller.
   *
   * @param constants
   */
  public void configure();

  /**
   * Updates the position controller's values.
   *
   * @param values
   */
  public void update(PositionControllerIOValues values);

  /**
   * Sets the position controller position.
   *
   * @param positionRotations
   */
  public void setPosition(double positionRotations);

  /**
   * Sets the position setpoint.
   *
   * @param positionRotations
   * @param velocityRotationsPerSecond
   */
  public void setSetpoint(double positionRotations, double velocityRotationsPerSecond);
}
