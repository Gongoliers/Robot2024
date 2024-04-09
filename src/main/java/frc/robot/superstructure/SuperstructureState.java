package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.arm.Arm;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeConstants.RollerConstants;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.ShooterConstants.FlywheelConstants;
import frc.robot.shooter.ShooterConstants.SerializerConstants;
import frc.robot.superstructure.SuperstructureConstants.ShoulderAngleConstants;
import java.util.Objects;

/** Represents the state of the superstructure. */
public record SuperstructureState(
    State shoulderRotations,
    double rollerVelocityRotationsPerSecond,
    double flywheelVelocityRotationsPerSecond,
    double serializerVelocityRotationsPerSecond) {

  public static final SuperstructureState INITIAL =
      new SuperstructureState(ShoulderAngleConstants.INITIAL, 0, 0, 0);

  public static final SuperstructureState STOW =
      new SuperstructureState(ShoulderAngleConstants.STOW, 0, 0, 0);

  public static final SuperstructureState INTAKE_POSITION =
      new SuperstructureState(ShoulderAngleConstants.STOW, 0, 0, 0);

  public static final SuperstructureState INTAKE =
      new SuperstructureState(
          ShoulderAngleConstants.STOW,
          RollerConstants.INTAKE_VELOCITY,
          0,
          SerializerConstants.INTAKE_VELOCITY);

  public static final SuperstructureState PULL = new SuperstructureState(ShoulderAngleConstants.STOW, 0, 0, SerializerConstants.PULL_VELOCITY);

  public static final SuperstructureState EJECT_POSITION = new SuperstructureState(ShoulderAngleConstants.EJECT, 0, 0, 0);

  public static final SuperstructureState EJECT = new SuperstructureState(ShoulderAngleConstants.EJECT, 0, 0, SerializerConstants.PULL_VELOCITY);

  public static final SuperstructureState SPEAKER_SHOOT =
      new SuperstructureState(
          ShoulderAngleConstants.SHOOT,
          0,
          FlywheelConstants.SPEAKER_VELOCITY,
          SerializerConstants.SERIALIZE_VELOCITY);

  public static final SuperstructureState PASS_SPIN =
      new SuperstructureState(
          ShoulderAngleConstants.STOW, 0, FlywheelConstants.PASS_VELOCITY, 0);

  public static final SuperstructureState PASS_SHOOT =
      new SuperstructureState(
          ShoulderAngleConstants.STOW,
          0,
          FlywheelConstants.PASS_VELOCITY,
          SerializerConstants.SERIALIZE_VELOCITY);

  public static final SuperstructureState AMP_POSITION =
      new SuperstructureState(ShoulderAngleConstants.AMP, 0, 0, 0);

  public static final SuperstructureState AMP_SPIN =
      new SuperstructureState(
          ShoulderAngleConstants.AMP, 0, FlywheelConstants.AMP_VELOCITY, 0);

  public static final SuperstructureState AMP_SHOOT =
      new SuperstructureState(
          ShoulderAngleConstants.AMP,
          0,
          FlywheelConstants.AMP_VELOCITY,
          SerializerConstants.SERIALIZE_VELOCITY);

  /**
   * Creates a new superstructure state.
   *
   * @param shoulderRotations
   * @param rollerVelocityRotationsPerSecond
   * @param flywheelVelocityRotationsPerSecond
   * @param serializerVelocityRotationsPerSecond
   */
  public SuperstructureState {
    Objects.requireNonNull(shoulderRotations);
    Objects.requireNonNull(rollerVelocityRotationsPerSecond);
    Objects.requireNonNull(flywheelVelocityRotationsPerSecond);
    Objects.requireNonNull(serializerVelocityRotationsPerSecond);
  }

  /**
   * Creates a new superstructure state.
   *
   * @param shoulderAngle
   * @param rollerVelocityRotationsPerSecond
   * @param flywheelVelocityRotationsPerSecond
   * @param serializerVelocityRotationsPerSecond
   */
  public SuperstructureState(
      Rotation2d shoulderAngle,
      double rollerVelocityRotationsPerSecond,
      double flywheelVelocityRotationsPerSecond,
      double serializerVelocityRotationsPerSecond) {
    this(
        new State(shoulderAngle.getRotations(), 0),
        rollerVelocityRotationsPerSecond,
        flywheelVelocityRotationsPerSecond,
        serializerVelocityRotationsPerSecond);
  }

  /**
   * Returns true if at the superstructure goal.
   *
   * @param goal
   * @return true if at the superstructure goal.
   */
  public boolean atGoal(SuperstructureState goal) {
    return Arm.getInstance().atGoal()
        && Intake.getInstance().atGoal()
        && Shooter.getInstance().atGoal();
  }
}
