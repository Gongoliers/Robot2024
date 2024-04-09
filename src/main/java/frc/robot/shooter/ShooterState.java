package frc.robot.shooter;

import edu.wpi.first.math.MathUtil;
import java.util.Objects;

public record ShooterState(
    double flywheelVelocityRotationsPerSecond, double serializerVelocityRotationsPerSecond) {

  public static final ShooterState IDLE = new ShooterState(0, 0);

  public static final ShooterState INTAKE = new ShooterState(0, 34);

  public static final ShooterState PULL = new ShooterState(0, -20);

  public static final ShooterState EJECT = new ShooterState(0, -44);

  public static final ShooterState SPEAKER_READY = new ShooterState(44, 0);

  public static final ShooterState SPEAKER_SHOOT = new ShooterState(44, 20);

  public static final ShooterState PASS_READY = new ShooterState(44, 0);

  public static final ShooterState PASS_SHOOT = new ShooterState(44, 20);

  public static final ShooterState AMP_SHOOT = new ShooterState(20, 20);

  public ShooterState {
    Objects.requireNonNull(flywheelVelocityRotationsPerSecond);
    Objects.requireNonNull(serializerVelocityRotationsPerSecond);
  }

  public boolean at(ShooterState other) {
    return MathUtil.isNear(
            flywheelVelocityRotationsPerSecond, other.flywheelVelocityRotationsPerSecond, 2.5)
        && MathUtil.isNear(
            serializerVelocityRotationsPerSecond, other.serializerVelocityRotationsPerSecond, 2.5);
  }
}
