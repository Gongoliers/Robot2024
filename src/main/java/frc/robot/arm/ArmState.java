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

  public static final ArmState STOW = new ArmState(ShoulderMotorConstants.MINIMUM_ANGLE, WristMotorConstants.MAXIMUM_ANGLE);

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
    this(
        new State(shoulder.getRotations(), 0.0),
        new State(wrist.getRotations(), 0.0));
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
    boolean atShoulder =
        MathUtil.isNear(
            this.shoulder().position,
            other.shoulder().position,
            ShoulderMotorConstants.TOLERANCE.getRotations());
    boolean atWrist =
        MathUtil.isNear(
            this.wrist().position,
            other.wrist().position,
            WristMotorConstants.TOLERANCE.getRotations());

    return atShoulder && atWrist;
  }

  public ArmState nextSetpoint(ArmState goal) {
    State nextShoulderState =
        ShoulderMotorConstants.MOTION_PROFILE.calculate(
            RobotConstants.PERIODIC_DURATION, this.shoulder, goal.shoulder);

    State nextWristState =
        WristMotorConstants.MOTION_PROFILE.calculate(
            RobotConstants.PERIODIC_DURATION, this.wrist, goal.wrist);

    return new ArmState(nextShoulderState, nextWristState);
  }
}
