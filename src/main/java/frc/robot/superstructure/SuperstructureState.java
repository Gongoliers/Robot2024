package frc.robot.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import java.util.Objects;

/** Represents the state of the superstructure. */
public record SuperstructureState(
    State shoulderAngleRotations,
    State wristAngleRotations,
    State pivotAngleRotations,
    double rollerVelocityRotationsPerSecond,
    double flywheelVelocityRotationsPerSecond,
    double serializerVelocityRotationsPerSecond) {

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
    Objects.requireNonNull(pivotAngleRotations);
    Objects.requireNonNull(rollerVelocityRotationsPerSecond);
    Objects.requireNonNull(flywheelVelocityRotationsPerSecond);
    Objects.requireNonNull(serializerVelocityRotationsPerSecond);
  }

  /**
   * Creates a new superstructure state.
   *
   * @param shoulderAngle
   * @param wristAngle
   * @param pivotAngle
   * @param rollerVelocityRotationsPerSecond
   * @param flywheelVelocityRotationsPerSecond
   * @param serializerVelocityRotationsPerSecond
   */
  public SuperstructureState(
      Rotation2d shoulderAngle,
      Rotation2d wristAngle,
      Rotation2d pivotAngle,
      double rollerVelocityRotationsPerSecond,
      double flywheelVelocityRotationsPerSecond,
      double serializerVelocityRotationsPerSecond) {
    this(
        new State(shoulderAngle.getRotations(), 0.0),
        new State(wristAngle.getRotations(), 0.0),
        new State(pivotAngle.getRotations(), 0.0),
        rollerVelocityRotationsPerSecond,
        flywheelVelocityRotationsPerSecond,
        serializerVelocityRotationsPerSecond);
  }

  /**
   * Creates a new superstructure state.
   *
   * @param shoulderAngle
   * @param wristAngle
   * @param pivotAngle
   */
  public SuperstructureState(
      Rotation2d shoulderAngle, Rotation2d wristAngle, Rotation2d pivotAngle) {
    this(shoulderAngle, wristAngle, pivotAngle, 0.0, 0.0, 0.0);
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

  /**
   * Returns true if at the intake pivot angle goal.
   *
   * @param goal
   * @return true if at the intake pivot angle goal.
   */
  public boolean atPivotAngleGoal(SuperstructureState goal) {
    return MathUtil.isNear(
        this.pivotAngleRotations().position,
        goal.pivotAngleRotations().position,
        SuperstructureConstants.IntakePivotAngleConstants.TOLERANCE.getRotations());
  }
}
