package frc.robot.arm;

public interface LimitSwitchIO {

  public static class LimitSwitchIOValues {
    /** If true, the limit switch is pressed. */
    public boolean isPressed = false;
  }

  /** Configures the limit switch. */
  public void configure();

  /**
   * Updates the limit switch's values.
   *
   * @param values
   */
  public void update(LimitSwitchIOValues values);
}
