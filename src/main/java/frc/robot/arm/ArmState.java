package frc.robot.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.RobotConstants;
import frc.robot.arm.ArmConstants.ElbowMotorConstants;
import frc.robot.arm.ArmConstants.ShoulderMotorConstants;
import java.util.Objects;

/** State of the arm. */
public record ArmState(State shoulder, State elbow, State wrist) {

  public static final ArmState STOW =
      new ArmState(
          Rotation2d.fromDegrees(12.5),
          Rotation2d.fromDegrees(180 - 18.125),
          Rotation2d.fromDegrees(0));

  public static final ArmState SHOOT = STOW.withElbow(Rotation2d.fromDegrees(180 - 35));

  public static final ArmState INTAKE =
      new ArmState(
          Rotation2d.fromDegrees(90 - 25.6),
          Rotation2d.fromDegrees(-16.325),
          Rotation2d.fromDegrees(-50));

  public static final ArmState AMP =
      new ArmState(
          Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(0));

  /**
   * Creates an arm state.
   *
   * @param shoulder the shoulder's state.
   * @param elbow the elbow's state.
   * @param wrist the wrist's state.
   */
  public ArmState {
    Objects.requireNonNull(shoulder);
    Objects.requireNonNull(elbow);
    Objects.requireNonNull(wrist);
  }

  /**
   * Creates an arm state.
   *
   * @param shoulder the shoulder's rotation.
   * @param elbow the elbow's rotation.
   * @param wrist the wrist's rotation.
   */
  public ArmState(Rotation2d shoulder, Rotation2d elbow, Rotation2d wrist) {
    this(
        new State(shoulder.getRotations(), 0.0),
        new State(elbow.getRotations(), 0.0),
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
    return new ArmState(newShoulder, elbow, wrist);
  }

  /**
   * Copies this arm state with a new elbow rotation.
   *
   * @param newElbow the new elbow rotation.
   * @return a copy of this arm state with a new elbow rotation.
   */
  public ArmState withElbow(Rotation2d newElbow) {
    return withElbow(new State(newElbow.getRotations(), 0.0));
  }

  /**
   * Copies this arm state with a new elbow state.
   *
   * @param newElbow the new elbow state.
   * @return a copy of this arm state with a new elbow state.
   */
  public ArmState withElbow(State newElbow) {
    return new ArmState(shoulder, newElbow, wrist);
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
    return new ArmState(shoulder, elbow, newWrist);
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
    boolean atElbow =
        MathUtil.isNear(
            this.elbow().position,
            other.elbow().position,
            ElbowMotorConstants.TOLERANCE.getRotations());

    return atShoulder && atElbow;
  }

  public ArmState nextSetpoint(ArmState goal) {
    State nextShoulderState =
        ShoulderMotorConstants.MOTION_PROFILE.calculate(
            RobotConstants.PERIODIC_DURATION, this.shoulder, goal.shoulder);

    State nextElbowState =
        ElbowMotorConstants.MOTION_PROFILE.calculate(
            RobotConstants.PERIODIC_DURATION, this.elbow, goal.elbow);

    return new ArmState(nextShoulderState, nextElbowState, new State(0.0, 0.0));
  }
}
