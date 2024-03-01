package frc.robot.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.RobotConstants;
import frc.robot.arm.ArmConstants.ShoulderMotorConstants;
import frc.robot.arm.ArmConstants.WristMotorConstants;
import java.util.Objects;

/** State of the arm. */
public record ArmState(State shoulder, State wrist) {

  public static final ArmState INIT =
      new ArmState(Rotation2d.fromDegrees(52.5), Rotation2d.fromDegrees(-35));

  public static final ArmState STOW =
      new ArmState(ShoulderMotorConstants.MINIMUM_ANGLE, WristMotorConstants.MAXIMUM_ANGLE);
  public static final ArmState SHOOT = STOW.withWrist(Rotation2d.fromDegrees(23.265));
  public static final ArmState INTAKE = STOW.withWrist(Rotation2d.fromDegrees(6.81));

  public static final ArmState AMP =
      new ArmState(ShoulderMotorConstants.MAXIMUM_ANGLE, Rotation2d.fromDegrees(0));

  /**
   * Creates an arm state.
   *
   * @param shoulder the shoulder's state.
   * @param wrist the wrist's state.
   */
  public ArmState {
    Objects.requireNonNull(shoulder);
    Objects.requireNonNull(wrist);
  }

  /**
   * Creates an arm state.
   *
   * @param shoulder the shoulder's rotation.
   * @param wrist the wrist's rotation.
   */
  public ArmState(Rotation2d shoulder, Rotation2d wrist) {
    this(new State(shoulder.getRotations(), 0.0), new State(wrist.getRotations(), 0.0));
  }

  /**
   * Copies this arm state with a new shoulder rotation.
   *
   * @param newShoulder the new shoulder rotation.
   * @return a copy of this arm state with a new shoulder rotation.
   */
  public ArmState withShoulder(Rotation2d newShoulder) {
    return withShoulder(new State(newShoulder.getRotations(), 0.0));
  }

  /**
   * Copies this arm state with a new shoulder state.
   *
   * @param newShoulder the new shoulder state.
   * @return a copy of this arm state with a new shoulder state.
   */
  public ArmState withShoulder(State newShoulder) {
    return new ArmState(newShoulder, wrist);
  }

  /**
   * Copies this arm state with a new wrist rotation.
   *
   * @param newWrist the new wrist rotation.
   * @return a copy of this arm state with a new wrist rotation.
   */
  public ArmState withWrist(Rotation2d newWrist) {
    return withWrist(new State(newWrist.getRotations(), 0.0));
  }

  /**
   * Copies this arm state with a new wrist state.
   *
   * @param newWrist the new wrist state.
   * @return a copy of this arm state with a new wrist state.
   */
  public ArmState withWrist(State newWrist) {
    return new ArmState(shoulder, newWrist);
  }

  /**
   * Returns true if the arm states are equal.
   *
   * @param other the other arm state.
   * @return true if the arm states are equal.
   */
  public boolean at(ArmState other) {
    return atShoulderGoal(other) && atWristGoal(other);
  }

  /**
   * Returns true if the arm is at its shoulder goal.
   *
   * @param goal goal state of the arm.
   * @return true if the arm is at its shoulder goal.
   */
  public boolean atShoulderGoal(ArmState goal) {
    return MathUtil.isNear(
        this.shoulder().position,
        goal.shoulder().position,
        ShoulderMotorConstants.TOLERANCE.getRotations());
  }

  /**
   * Returns the next setpoint involving shoulder-only movement.
   *
   * @param goal the arm's goal state.
   * @return the next setpoint involving shoulder-only movement.
   */
  public ArmState nextShoulderSetpoint(ArmState goal) {
    return this.withShoulder(
        ShoulderMotorConstants.MOTION_PROFILE.calculate(
            RobotConstants.PERIODIC_DURATION, this.shoulder, goal.shoulder));
  }

  /**
   * Returns true if the arm is at its wrist goal.
   *
   * @param goal goal state of the arm.
   * @return true if the arm is at its wrist goal.
   */
  public boolean atWristGoal(ArmState goal) {
    return MathUtil.isNear(
        this.wrist().position, goal.wrist().position, WristMotorConstants.TOLERANCE.getRotations());
  }

  /**
   * Returns the next setpoint involving wrist-only movement.
   *
   * @param goal the arm's goal state.
   * @return the next setpoint involving wrist-only movement.
   */
  public ArmState nextWristSetpoint(ArmState goal) {
    return this.withWrist(
        WristMotorConstants.MOTION_PROFILE.calculate(
            RobotConstants.PERIODIC_DURATION, this.wrist, goal.wrist));
  }

  /**
   * Returns the next overall setpoint of the arm's movement.
   *
   * @param goal the arm's goal state.
   * @return the next overall setpoint.
   */
  public ArmState nextSetpoint(ArmState goal) {
    boolean shooterOnBottom = Rotation2d.fromRotations(wrist.position).getDegrees() < 0.0;
    boolean shooterNeedsToBeOnTop =
        Rotation2d.fromRotations(goal.wrist.position).getDegrees() > 0.0;
    boolean shooterOnWrongSide = shooterOnBottom && shooterNeedsToBeOnTop;

    if (shooterOnWrongSide && !atWristGoal(goal)) {
      return nextWristSetpoint(goal);
    }

    if (!atShoulderGoal(goal)) {
      return nextShoulderSetpoint(goal);
    } else if (!atWristGoal(goal)) {
      return nextWristSetpoint(goal);
    }

    return this;
  }
}
