package frc.robot.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.RobotConstants;
import frc.robot.superstructure.SuperstructureConstants.PivotAngleConstants;
import frc.robot.superstructure.SuperstructureConstants.ShoulderAngleConstants;
import frc.robot.superstructure.SuperstructureConstants.WristAngleConstants;
import java.util.Objects;

/** Represents the state of the superstructure. */
public record SuperstructureState(
    State shoulderAngleRotations,
    State wristAngleRotations,
    State pivotAngleRotations,
    double rollerVelocityRotationsPerSecond,
    double flywheelVelocityRotationsPerSecond,
    double serializerVelocityRotationsPerSecond) {

  public static final SuperstructureState INITIAL =
      new SuperstructureState(
          Rotation2d.fromDegrees(52.5), Rotation2d.fromDegrees(-35), Rotation2d.fromDegrees(86));

  public static final SuperstructureState STOW =
      new SuperstructureState(
          Rotation2d.fromDegrees(29.5), Rotation2d.fromDegrees(85.98), Rotation2d.fromDegrees(86));

  public static final SuperstructureState INTAKE =
      new SuperstructureState(
          Rotation2d.fromDegrees(29.5), Rotation2d.fromDegrees(4), Rotation2d.fromDegrees(-48));

  public static final SuperstructureState SHOOT =
      new SuperstructureState(
          Rotation2d.fromDegrees(29.5), Rotation2d.fromDegrees(18), Rotation2d.fromDegrees(-48));

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
        SuperstructureConstants.PivotAngleConstants.TOLERANCE.getRotations());
  }

  public boolean at(SuperstructureState other) {
    return atShoulderAngleGoal(other) && atWristAngleGoal(other) && atPivotAngleGoal(other);
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

    State nextPivotSetpoint =
        PivotAngleConstants.MOTION_PROFILE.calculate(
            RobotConstants.PERIODIC_DURATION,
            setpoint.pivotAngleRotations(),
            goal.pivotAngleRotations());

    return new SuperstructureState(
        nextShoulderSetpoint,
        nextWristSetpoint,
        nextPivotSetpoint,
        goal.rollerVelocityRotationsPerSecond(),
        goal.flywheelVelocityRotationsPerSecond(),
        goal.serializerVelocityRotationsPerSecond());
  }
}
