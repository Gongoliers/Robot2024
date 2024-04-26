package frc.robot.shooter;

import edu.wpi.first.math.MathUtil;
import java.util.Objects;

/** Shooter state. */
public record ShooterState(
    double flywheelVelocityRotationsPerSecond, double serializerVelocityRotationsPerSecond) {

  /** Idling state. */
  public static final ShooterState IDLING = new ShooterState(0, 0);

  /** Intaking state. */
  public static final ShooterState INTAKING = new ShooterState(0, 34);

  /** Pulling state. */
  public static final ShooterState PULLING = new ShooterState(-20, -10);

  /** Ejecting state. */
  public static final ShooterState EJECTING = new ShooterState(0, -44);

  /** Subwoofer shooting state. */
  public static final ShooterState SUBWOOFER_SHOOTING = new ShooterState(60, 44);

  /** Skim shooting state. */
  public static final ShooterState SKIM_SHOOTING = new ShooterState(60, 44);

  /** Amp shooting state. */
  public static final ShooterState AMPING = new ShooterState(10, 20);

  /**
   * Shooter state.
   *
   * @param flywheelVelocityRotationsPerSecond flywheel velocity in rotations per second.
   * @param serializerVelocityRotationsPerSecond serializer velocity in rotations per second.
   */
  public ShooterState {
    Objects.requireNonNull(flywheelVelocityRotationsPerSecond);
    Objects.requireNonNull(serializerVelocityRotationsPerSecond);
  }

  /**
   * Returns true if this shooter state is at another shooter state.
   *
   * @param other another shooter state.
   * @return true if this shooter state is at another shooter state.
   */
  public boolean at(ShooterState other) {
    return MathUtil.isNear(
            flywheelVelocityRotationsPerSecond, other.flywheelVelocityRotationsPerSecond, 5.0)
        && MathUtil.isNear(
            serializerVelocityRotationsPerSecond, other.serializerVelocityRotationsPerSecond, 5.0);
  }
}
