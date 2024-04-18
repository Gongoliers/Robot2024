package frc.robot.shooter;

import edu.wpi.first.math.MathUtil;
import frc.robot.shooter.ShooterConstants.FlywheelConstants;
import frc.robot.shooter.ShooterConstants.SerializerConstants;
import java.util.Objects;

public record ShooterState(
    double flywheelVelocityRotationsPerSecond, double serializerVelocityRotationsPerSecond) {

  public static final ShooterState IDLE = new ShooterState(0, 0);

  public static final ShooterState INTAKE = new ShooterState(0, SerializerConstants.INTAKE_SPEED);

  public static final ShooterState PULL =
      new ShooterState(FlywheelConstants.PULL_SPEED, SerializerConstants.PULL_SPEED);

  public static final ShooterState EJECT = new ShooterState(0, SerializerConstants.EJECT_SPEED);

  public static final ShooterState SUBWOOFER =
      new ShooterState(FlywheelConstants.SPEAKER_SPEED, SerializerConstants.FAST_FEED_SPEED);

  public static final ShooterState PODIUM =
      new ShooterState(FlywheelConstants.PODIUM_SPEED, SerializerConstants.FAST_FEED_SPEED);

  public static final ShooterState LOB =
      new ShooterState(FlywheelConstants.LOB_SPEED, SerializerConstants.FAST_FEED_SPEED);

  public static final ShooterState SKIM =
      new ShooterState(FlywheelConstants.SKIM_SPEED, SerializerConstants.FAST_FEED_SPEED);

  public static final ShooterState AMP =
      new ShooterState(FlywheelConstants.AMP_SPEED, SerializerConstants.SLOW_FEED_SPEED);

  public ShooterState {
    Objects.requireNonNull(flywheelVelocityRotationsPerSecond);
    Objects.requireNonNull(serializerVelocityRotationsPerSecond);
  }

  public boolean at(ShooterState other) {
    return MathUtil.isNear(
            flywheelVelocityRotationsPerSecond, other.flywheelVelocityRotationsPerSecond, FlywheelConstants.TOLERANCE)
        && MathUtil.isNear(
            serializerVelocityRotationsPerSecond, other.serializerVelocityRotationsPerSecond, SerializerConstants.TOLERANCE);
  }
}
