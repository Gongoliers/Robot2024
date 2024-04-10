package frc.robot.shooter;

import edu.wpi.first.math.MathUtil;
import frc.robot.shooter.ShooterConstants.FlywheelConstants;
import frc.robot.shooter.ShooterConstants.SerializerConstants;

import java.util.Objects;

public record ShooterState(
    double flywheelVelocityRotationsPerSecond, double serializerVelocityRotationsPerSecond) {

  public static final ShooterState IDLE = new ShooterState(0, 0);

  public static final ShooterState INTAKE = new ShooterState(0, SerializerConstants.INTAKE_SPEED);

  public static final ShooterState PULL = new ShooterState(0, SerializerConstants.PULL_SPEED);

  public static final ShooterState EJECT = new ShooterState(0, SerializerConstants.EJECT_SPEED);

  public static final ShooterState SPEAKER_READY = new ShooterState(FlywheelConstants.SPEAKER_SPEED, 0);

  public static final ShooterState SPEAKER_SHOOT = new ShooterState(FlywheelConstants.SPEAKER_SPEED, SerializerConstants.FEED_SPEED);

  public static final ShooterState PASS_READY = new ShooterState(FlywheelConstants.PASS_SPEED, 0);

  public static final ShooterState PASS_SHOOT = new ShooterState(FlywheelConstants.PASS_SPEED, SerializerConstants.FEED_SPEED);

  public static final ShooterState AMP_SHOOT = new ShooterState(FlywheelConstants.AMP_SPEED, SerializerConstants.FEED_SPEED);

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
