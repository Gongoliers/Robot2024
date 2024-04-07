package frc.robot.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.RobotConstants;
import frc.robot.intake.IntakeConstants.RollerConstants;
import frc.robot.shooter.ShooterConstants.FlywheelConstants;
import frc.robot.shooter.ShooterConstants.SerializerConstants;
import frc.robot.superstructure.SuperstructureConstants.ShoulderAngleConstants;
import java.util.Objects;

/** Represents the state of the superstructure. */
public record SuperstructureState(
    State shoulderAngleRotations,
    boolean shoulderManual,
    double rollerVelocityRotationsPerSecond,
    double flywheelVelocityRotationsPerSecond,
    boolean rampFlywheelVelocity,
    double serializerVelocityRotationsPerSecond) {

  public static final SuperstructureState INITIAL =
      new SuperstructureState(ShoulderAngleConstants.INITIAL, false, 0, 0, false, 0);

  public static final SuperstructureState STOW =
      new SuperstructureState(ShoulderAngleConstants.STOW, false, 0, 0, false, 0);

  public static final SuperstructureState INTAKE_POSITION =
      new SuperstructureState(ShoulderAngleConstants.STOW, false, 0, 0, false, 0);

  public static final SuperstructureState INTAKE =
      new SuperstructureState(
          ShoulderAngleConstants.STOW,
          false,
          RollerConstants.INTAKE_VELOCITY,
          0,
          false,
          SerializerConstants.INTAKE_VELOCITY);

  public static final SuperstructureState PULL = new SuperstructureState(ShoulderAngleConstants.STOW, false, 0, 0, false, SerializerConstants.PULL_VELOCITY);

  public static final SuperstructureState EJECT_POSITION = new SuperstructureState(ShoulderAngleConstants.EJECT, false, 0, 0, false, 0);

  public static final SuperstructureState EJECT = new SuperstructureState(ShoulderAngleConstants.EJECT, false, 0, 0, false, SerializerConstants.PULL_VELOCITY);

  public static final SuperstructureState SPEAKER_SPIN =
      new SuperstructureState(
          ShoulderAngleConstants.SHOOT, false, 0, FlywheelConstants.SPEAKER_VELOCITY, true, 0);

  public static final SuperstructureState SPEAKER_SHOOT =
      new SuperstructureState(
          ShoulderAngleConstants.SHOOT,
          false,
          0,
          FlywheelConstants.SPEAKER_VELOCITY,
          false,
          SerializerConstants.SERIALIZE_VELOCITY);

  public static final SuperstructureState PASS_SPIN =
      new SuperstructureState(
          ShoulderAngleConstants.STOW, false, 0, FlywheelConstants.PASS_VELOCITY, false, 0);

  public static final SuperstructureState PASS_SHOOT =
      new SuperstructureState(
          ShoulderAngleConstants.STOW,
          false,
          0,
          FlywheelConstants.PASS_VELOCITY,
          false,
          SerializerConstants.SERIALIZE_VELOCITY);

  public static final SuperstructureState AMP_POSITION =
      new SuperstructureState(ShoulderAngleConstants.AMP, false, 0, 0, false, 0);

  public static final SuperstructureState AMP_SPIN =
      new SuperstructureState(
          ShoulderAngleConstants.AMP, false, 0, FlywheelConstants.AMP_VELOCITY, false, 0);

  public static final SuperstructureState AMP_SHOOT =
      new SuperstructureState(
          ShoulderAngleConstants.AMP,
          false,
          0,
          FlywheelConstants.AMP_VELOCITY,
          false,
          SerializerConstants.SERIALIZE_VELOCITY);

  public static final SuperstructureState MANUAL = new SuperstructureState(ShoulderAngleConstants.STOW, true, 0, 0, false, 0);

  /**
   * Creates a new superstructure state.
   *
   * @param shoulderAngleRotations
   * @param shoulderManual
   * @param rollerVelocityRotationsPerSecond
   * @param flywheelVelocityRotationsPerSecond
   * @param rampFlywheelVelocity
   * @param serializerVelocityRotationsPerSecond
   */
  public SuperstructureState {
    Objects.requireNonNull(shoulderAngleRotations);
    Objects.requireNonNull(shoulderManual);
    Objects.requireNonNull(rollerVelocityRotationsPerSecond);
    Objects.requireNonNull(flywheelVelocityRotationsPerSecond);
    Objects.requireNonNull(rampFlywheelVelocity);
    Objects.requireNonNull(serializerVelocityRotationsPerSecond);
  }

  /**
   * Creates a new superstructure state.
   *
   * @param shoulderAngle
   * @param shoulderManual
   * @param rollerVelocityRotationsPerSecond
   * @param flywheelVelocityRotationsPerSecond
   * @param rampFlywheelVelocity
   * @param serializerVelocityRotationsPerSecond
   */
  public SuperstructureState(
      Rotation2d shoulderAngle,
      boolean shoulderManual,
      double rollerVelocityRotationsPerSecond,
      double flywheelVelocityRotationsPerSecond,
      boolean rampFlywheelVelocity,
      double serializerVelocityRotationsPerSecond) {
    this(
        new State(shoulderAngle.getRotations(), 0),
        shoulderManual,
        rollerVelocityRotationsPerSecond,
        flywheelVelocityRotationsPerSecond,
        rampFlywheelVelocity,
        serializerVelocityRotationsPerSecond);
  }

  /**
   * Returns true if at the shoulder angle goal.
   *
   * @param goal
   * @return true if at the shoulder angle goal.
   */
  public boolean atShoulderAngleGoal(SuperstructureState goal) {
    return MathUtil.isNear(
        this.shoulderAngleRotations().position,
        goal.shoulderAngleRotations().position,
        SuperstructureConstants.ShoulderAngleConstants.TOLERANCE.getRotations());
  }

  /**
   * Returns true if at the roller velocity goal.
   *
   * @param goal
   * @return true if at the roller velocity goal.
   */
  public boolean atRollerVelocityGoal(SuperstructureState goal) {
    return MathUtil.isNear(
        this.rollerVelocityRotationsPerSecond(),
        goal.rollerVelocityRotationsPerSecond(),
        RollerConstants.SPEED_TOLERANCE);
  }

  /**
   * Returns true if at the flywheel velocity goal.
   *
   * @param goal
   * @return true if at the flywheel velocity goal.
   */
  public boolean atFlywheelVelocityGoal(SuperstructureState goal) {
    return MathUtil.isNear(
        this.flywheelVelocityRotationsPerSecond(),
        goal.flywheelVelocityRotationsPerSecond(),
        FlywheelConstants.SPEED_TOLERANCE);
  }

  /**
   * Returns true if at the serializer velocity goal.
   *
   * @param goal
   * @return true if at the serializer velocity goal.
   */
  public boolean atSerializerVelocityGoal(SuperstructureState goal) {
    return MathUtil.isNear(
        this.serializerVelocityRotationsPerSecond(),
        goal.serializerVelocityRotationsPerSecond(),
        SerializerConstants.SPEED_TOLERANCE);
  }

  /**
   * Returns true if at the superstructure goal.
   *
   * @param goal
   * @return true if at the superstructure goal.
   */
  public boolean atGoal(SuperstructureState goal) {
    return atShoulderAngleGoal(goal)
        && atRollerVelocityGoal(goal)
        && atFlywheelVelocityGoal(goal)
        && atSerializerVelocityGoal(goal);
  }

  /**
   * Calculates the next setpoint.
   *
   * @param setpoint the previous setpoint.
   * @param goal the goal.
   * @return the next setpoint.
   */
  public static SuperstructureState nextSetpoint(
      SuperstructureState setpoint, SuperstructureState goal) {
    State nextShoulderSetpoint =
        ShoulderAngleConstants.MOTION_PROFILE.calculate(
            RobotConstants.PERIODIC_DURATION,
            setpoint.shoulderAngleRotations(),
            goal.shoulderAngleRotations());

    double accelerationLimitedFlywheelVelocity =
        FlywheelConstants.ACCELERATION_LIMITER.calculate(goal.flywheelVelocityRotationsPerSecond());

    double nextFlywheelVelocitySetpoint =
        goal.rampFlywheelVelocity()
            ? accelerationLimitedFlywheelVelocity
            : goal.flywheelVelocityRotationsPerSecond();

    return new SuperstructureState(
        nextShoulderSetpoint,
        goal.shoulderManual(),
        goal.rollerVelocityRotationsPerSecond(),
        nextFlywheelVelocitySetpoint,
        goal.rampFlywheelVelocity(),
        goal.serializerVelocityRotationsPerSecond());
  }
}
