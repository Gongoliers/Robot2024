package frc.robot.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.RobotConstants;
import frc.robot.shooter.ShooterConstants.FlywheelConstants;
import frc.robot.shooter.ShooterConstants.SerializerConstants;
import frc.robot.superstructure.SuperstructureConstants.ShoulderAngleConstants;
import java.util.Objects;

/** Represents the state of the superstructure. */
public record SuperstructureState(
    State shoulderAngleRotations,
    double rollerVelocityRotationsPerSecond,
    double flywheelVelocityRotationsPerSecond,
    double serializerVelocityRotationsPerSecond) {

  public static final SuperstructureState INITIAL =
      new SuperstructureState(ShoulderAngleConstants.INITIAL);

  public static final SuperstructureState STOW =
      new SuperstructureState(ShoulderAngleConstants.STOW);

  public static final SuperstructureState INTAKE =
      new SuperstructureState(ShoulderAngleConstants.STOW);

  public static final SuperstructureState SHOOT =
      new SuperstructureState(ShoulderAngleConstants.STOW);

  public static final SuperstructureState AMP = new SuperstructureState(ShoulderAngleConstants.AMP);

  public static final SuperstructureState AMP_SHOOT =
      AMP.withFlywheelVelocity(FlywheelConstants.AMP_VELOCITY);

  public static final SuperstructureState AMP_SERIALIZE =
      AMP_SHOOT.withSerializerVelocity(SerializerConstants.AMP_SERIALIZE_VELOCITY);

  /**
   * Creates a new superstructure state.
   *
   * @param shoulderAngleRotations
   * @param intakeRollerVelocityRotationsPerSecond
   * @param flywheelVelocityRotationsPerSecond
   * @param serializerVelocityRotationsPerSecond
   */
  public SuperstructureState {
    Objects.requireNonNull(shoulderAngleRotations);
    Objects.requireNonNull(rollerVelocityRotationsPerSecond);
    Objects.requireNonNull(flywheelVelocityRotationsPerSecond);
    Objects.requireNonNull(serializerVelocityRotationsPerSecond);
  }

  /**
   * Creates a new superstructure state.
   *
   * @param shoulderAngle
   * @param wristAngle
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
        new State(shoulderAngle.getRotations(), 0.0),
        rollerVelocityRotationsPerSecond,
        flywheelVelocityRotationsPerSecond,
        serializerVelocityRotationsPerSecond);
  }

  /**
   * Creates a new superstructure state.
   *
   * @param shoulderAngle
   * @param wristAngle
   */
  public SuperstructureState(Rotation2d shoulderAngle) {
    this(shoulderAngle, 0.0, 0.0, 0.0);
  }

  public SuperstructureState withShoulderAngle(Rotation2d newShoulderAngle) {
    return new SuperstructureState(
        new State(newShoulderAngle.getRotations(), 0.0),
        rollerVelocityRotationsPerSecond,
        flywheelVelocityRotationsPerSecond,
        serializerVelocityRotationsPerSecond);
  }

  public SuperstructureState withShoulderAngleOf(SuperstructureState other) {
    return new SuperstructureState(
        other.shoulderAngleRotations,
        rollerVelocityRotationsPerSecond,
        flywheelVelocityRotationsPerSecond,
        serializerVelocityRotationsPerSecond);
  }

  public SuperstructureState withRollerVelocity(double newRollerVelocityRotationsPerSecond) {
    return new SuperstructureState(
        shoulderAngleRotations,
        newRollerVelocityRotationsPerSecond,
        flywheelVelocityRotationsPerSecond,
        serializerVelocityRotationsPerSecond);
  }

  public SuperstructureState withRollerVelocityOf(SuperstructureState other) {
    return new SuperstructureState(
        shoulderAngleRotations,
        other.rollerVelocityRotationsPerSecond,
        flywheelVelocityRotationsPerSecond,
        serializerVelocityRotationsPerSecond);
  }

  public SuperstructureState withFlywheelVelocity(double newFlywheelVelocityRotationsPerSecond) {
    return new SuperstructureState(
        shoulderAngleRotations,
        rollerVelocityRotationsPerSecond,
        newFlywheelVelocityRotationsPerSecond,
        serializerVelocityRotationsPerSecond);
  }

  public SuperstructureState withFlywheelVelocityOf(SuperstructureState other) {
    return new SuperstructureState(
        shoulderAngleRotations,
        rollerVelocityRotationsPerSecond,
        other.flywheelVelocityRotationsPerSecond,
        serializerVelocityRotationsPerSecond);
  }

  public SuperstructureState withSerializerVelocity(
      double newSerializerVelocityRotationsPerSecond) {
    return new SuperstructureState(
        shoulderAngleRotations,
        rollerVelocityRotationsPerSecond,
        flywheelVelocityRotationsPerSecond,
        newSerializerVelocityRotationsPerSecond);
  }

  public SuperstructureState withSerializerVelocityOf(SuperstructureState other) {
    return new SuperstructureState(
        shoulderAngleRotations,
        rollerVelocityRotationsPerSecond,
        flywheelVelocityRotationsPerSecond,
        other.serializerVelocityRotationsPerSecond);
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
   * Returns true if at the shoulder angle.
   *
   * @param other
   * @return true if at the shoulder angle.
   */
  public boolean atShoulderAngle(Rotation2d other) {
    return MathUtil.isNear(
        this.shoulderAngleRotations().position,
        other.getRotations(),
        SuperstructureConstants.ShoulderAngleConstants.TOLERANCE.getRotations());
  }

  public boolean at(SuperstructureState other) {
    return atShoulderAngleGoal(other);
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

    return new SuperstructureState(
        nextShoulderSetpoint,
        goal.rollerVelocityRotationsPerSecond(),
        goal.flywheelVelocityRotationsPerSecond(),
        goal.serializerVelocityRotationsPerSecond());
  }
}
