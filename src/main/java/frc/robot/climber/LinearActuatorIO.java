package frc.robot.climber;

/** Linear actuator hardware interface. */
public interface LinearActuatorIO {

  /** Values for the linear actuator hardware interface. */
  public static class LinearActuatorIOValues {
    /** Position of the linear actuator in percent. */
    public double positionPercent = 0.0;
  }

  /** Configures the linear acutator. */
  public void configure();

  /**
   * Updates the linear actuator's values.
   *
   * @param values
   */
  public void update(LinearActuatorIOValues values);

  /**
   * Sets the linear actuator's position.
   *
   * @param positionPercent the linear actuator's position.
   */
  public void setPosition(double positionPercent);

  /**
   * Sets the linear actuator's setpoint.
   *
   * @param positionPercent the linear actuator's setpoint.
   */
  public void setSetpoint(double positionPercent);
}
