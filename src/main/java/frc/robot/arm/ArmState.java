package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.RobotConstants;
import frc.robot.arm.ArmConstants.ShoulderMotorConstants;
import java.util.Objects;

/** State of the arm. */
public record ArmState(State shoulder, State elbow, State wrist) {

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
        new State(shoulder.getRotations(), 0),
        new State(elbow.getRotations(), 0),
        new State(wrist.getRotations(), 0));
  }

  /**
   * Copies this arm state with a new shoulder rotation.
   *
   * @param newShoulder the new shoulder rotation.
   * @return a copy of this arm state with a new shoulder rotation.
   */
  public ArmState withShoulder(Rotation2d newShoulder) {
    return withShoulder(new State(newShoulder.getRotations(), 0));
  }

  /**
   * Copies this arm state with a new shoulder state.
   *
   * @param newShoulder the new shoulder state.
   * @return a copy of this arm state with a new shoulder rotation.
   */
  public ArmState withShoulder(State newShoulder) {
    return new ArmState(newShoulder, elbow, wrist);
  }

  public ArmState nextSetpoint(ArmState goal) {
    State nextShoulderState =
        ShoulderMotorConstants.MOTION_PROFILE.calculate(
            RobotConstants.PERIODIC_DURATION, this.shoulder, goal.shoulder);

    return withShoulder(nextShoulderState);
  }
}
