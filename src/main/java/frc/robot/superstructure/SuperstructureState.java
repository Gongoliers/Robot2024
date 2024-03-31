package frc.robot.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.RobotConstants;
import frc.robot.superstructure.SuperstructureConstants.ShoulderAngleConstants;
import frc.robot.superstructure.SuperstructureConstants.WristAngleConstants;
import java.util.Objects;

/** Represents the state of the superstructure. */
public record SuperstructureState(
    State shoulderAngleRotations,
    State wristAngleRotations,
    double rollerVelocityRotationsPerSecond,
    double flywheelVelocityRotationsPerSecond,
    double serializerVelocityRotationsPerSecond) {

  public static final SuperstructureState INITIAL =
      new SuperstructureState(
          ShoulderAngleConstants.INITIAL, WristAngleConstants.INITIAL);

  public static final SuperstructureState STOW =
      new SuperstructureState(
          ShoulderAngleConstants.STOW, WristAngleConstants.STOW);

  public static final SuperstructureState INTAKE =
      new SuperstructureState(
          ShoulderAngleConstants.STOW, WristAngleConstants.INTAKE);

  public static final SuperstructureState SHOOT =
      new SuperstructureState(
          ShoulderAngleConstants.STOW, WristAngleConstants.SHOOT);

  public static final SuperstructureState AMP =
      new SuperstructureState(
          ShoulderAngleConstants.AMP, WristAngleConstants.AMP);

  /**
   * Creates a new superstructure state.
   *
   * @param shoulderAngleRotations
   * @param wristAngleRotations
   * @param intakePivotAngleRotations
   * @param intakeRollerVelocityRotationsPerSecond
   * @param flywheelVelocityRotationsPerSecond
   * @param serializerVelocityRotationsPerSecond
   */
  public SuperstructureState {
    Objects.requireNonNull(shoulderAngleRotations);
    Objects.requireNonNull(wristAngleRotations);
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
      Rotation2d wristAngle,
      double rollerVelocityRotationsPerSecond,
      double flywheelVelocityRotationsPerSecond,
      double serializerVelocityRotationsPerSecond) {
    this(
        new State(shoulderAngle.getRotations(), 0.0),
        new State(wristAngle.getRotations(), 0.0),
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
  public SuperstructureState(
      Rotation2d shoulderAngle, Rotation2d wristAngle) {
    this(shoulderAngle, wristAngle, 0.0, 0.0, 0.0);
  }

  public SuperstructureState withShoulderAngle(Rotation2d newShoulderAngle) {
    return new SuperstructureState(
        new State(newShoulderAngle.getRotations(), 0.0),
        wristAngleRotations,
        rollerVelocityRotationsPerSecond,
        flywheelVelocityRotationsPerSecond,
        serializerVelocityRotationsPerSecond);
  }

  public SuperstructureState withShoulderAngleOf(SuperstructureState other) {
    return new SuperstructureState(
        other.shoulderAngleRotations,
        wristAngleRotations,
        rollerVelocityRotationsPerSecond,
        flywheelVelocityRotationsPerSecond,
        serializerVelocityRotationsPerSecond);
  }

  public SuperstructureState withWristAngle(Rotation2d newWristAngle) {
    return new SuperstructureState(
        shoulderAngleRotations,
        new State(newWristAngle.getRotations(), 0.0),
        rollerVelocityRotationsPerSecond,
        flywheelVelocityRotationsPerSecond,
        serializerVelocityRotationsPerSecond);
  }

  public SuperstructureState withWristAngleOf(SuperstructureState other) {
    return new SuperstructureState(
        shoulderAngleRotations,
        other.wristAngleRotations,
        rollerVelocityRotationsPerSecond,
        flywheelVelocityRotationsPerSecond,
        serializerVelocityRotationsPerSecond);
  }

  public SuperstructureState withRollerVelocity(double newRollerVelocityRotationsPerSecond) {
    return new SuperstructureState(
        shoulderAngleRotations,
        wristAngleRotations,
        newRollerVelocityRotationsPerSecond,
        flywheelVelocityRotationsPerSecond,
        serializerVelocityRotationsPerSecond);
  }

  public SuperstructureState withRollerVelocityOf(SuperstructureState other) {
    return new SuperstructureState(
        shoulderAngleRotations,
        wristAngleRotations,
        other.rollerVelocityRotationsPerSecond,
        flywheelVelocityRotationsPerSecond,
        serializerVelocityRotationsPerSecond);
  }

  public SuperstructureState withFlywheelVelocity(double newFlywheelVelocityRotationsPerSecond) {
    return new SuperstructureState(
        shoulderAngleRotations,
        wristAngleRotations,
        rollerVelocityRotationsPerSecond,
        newFlywheelVelocityRotationsPerSecond,
        serializerVelocityRotationsPerSecond);
  }

  public SuperstructureState withFlywheelVelocityOf(SuperstructureState other) {
    return new SuperstructureState(
        shoulderAngleRotations,
        wristAngleRotations,
        rollerVelocityRotationsPerSecond,
        other.flywheelVelocityRotationsPerSecond,
        serializerVelocityRotationsPerSecond);
  }

  public SuperstructureState withSerializerVelocity(
      double newSerializerVelocityRotationsPerSecond) {
    return new SuperstructureState(
        shoulderAngleRotations,
        wristAngleRotations,
        rollerVelocityRotationsPerSecond,
        flywheelVelocityRotationsPerSecond,
        newSerializerVelocityRotationsPerSecond);
  }

  public SuperstructureState withSerializerVelocityOf(SuperstructureState other) {
    return new SuperstructureState(
        shoulderAngleRotations,
        wristAngleRotations,
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

  /**
   * Returns true if at the wrist angle goal.
   *
   * @param goal
   * @return true if at the wrist angle goal.
   */
  public boolean atWristAngleGoal(SuperstructureState goal) {
    return MathUtil.isNear(
        this.wristAngleRotations().position,
        goal.wristAngleRotations().position,
        SuperstructureConstants.WristAngleConstants.TOLERANCE.getRotations());
  }

  public boolean at(SuperstructureState other) {
    return atShoulderAngleGoal(other) && atWristAngleGoal(other);
  }

  public boolean wristStowed() {
    return wristAngleRotations.position > 0;
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

    State nextWristSetpoint =
        WristAngleConstants.MOTION_PROFILE.calculate(
            RobotConstants.PERIODIC_DURATION,
            setpoint.wristAngleRotations(),
            goal.wristAngleRotations());

    return new SuperstructureState(
        nextShoulderSetpoint,
        nextWristSetpoint,
        goal.rollerVelocityRotationsPerSecond(),
        goal.flywheelVelocityRotationsPerSecond(),
        goal.serializerVelocityRotationsPerSecond());
  }
}
