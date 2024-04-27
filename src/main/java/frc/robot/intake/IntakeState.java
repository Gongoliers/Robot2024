package frc.robot.intake;

import edu.wpi.first.math.MathUtil;
import java.util.Objects;

/** Intake state */
public record IntakeState(
    double frontRollerVelocityRotationsPerSecond, double backRollerVelocityRotationsPerSecond) {

  /** Idling state. */
  public static final IntakeState IDLE = new IntakeState(0, 0);

  /** Intaking state. */
  public static final IntakeState INTAKE = new IntakeState(34.0, 34.0);

  /** Ejecting state. */
  public static final IntakeState EJECTING = new IntakeState(-34.0, -34.0);

  /**
   * Intake state.
   *
   * @param frontRollerVelocityRotationsPerSecond front roller velocity in rotations per second.
   * @param backRollerVelocityRotationsPerSecond back roller velocity in rotations per second.
   */
  public IntakeState {
    Objects.requireNonNull(frontRollerVelocityRotationsPerSecond);
    Objects.requireNonNull(backRollerVelocityRotationsPerSecond);
  }

  /**
   * Returns true if this intake state is at another intake state.
   *
   * @param other another intake state.
   * @return true if this intake state is at another intake state.
   */
  public boolean at(IntakeState other) {
    return MathUtil.isNear(
            frontRollerVelocityRotationsPerSecond, other.frontRollerVelocityRotationsPerSecond, 1.0)
        && MathUtil.isNear(
            backRollerVelocityRotationsPerSecond, other.backRollerVelocityRotationsPerSecond, 1.0);
  }
}
