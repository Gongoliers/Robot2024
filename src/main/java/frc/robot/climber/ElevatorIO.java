package frc.robot.climber;

/** Elevator hardware interface. */
public interface ElevatorIO {

  /** Values for the elevator hardware interface. */
  public static class ElevatorIOValues {
    /** Position of the elevator in meters. */
    public double positionMeters;
  }

  /** Configures the elevator. */
  public void configure();

  /**
   * Updates the elevator's values.
   *
   * @param values
   */
  public void update(ElevatorIOValues values);

  /**
   * Sets the elevator's position.
   *
   * @param positionMeters the elevator's position.
   */
  public void setPosition(double positionMeters);

  /**
   * Sets the elevator's setpoint.
   *
   * @param positionMeters the elevator's setpoint.
   */
  public void setSetpoint(double positionMeters);
}
