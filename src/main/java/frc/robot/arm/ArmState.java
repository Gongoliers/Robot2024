package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Objects;

/** State of the arm. */
public record ArmState(Rotation2d shoulder, Rotation2d elbow, Rotation2d wrist) {

  /**
   * Creates an arm state.
   *
   * @param shoulder the shoulder's rotation.
   * @param elbow the elbow's rotation.
   * @param wrist the wrist's rotation.
   */
  public ArmState {
    Objects.requireNonNull(shoulder);
    Objects.requireNonNull(elbow);
    Objects.requireNonNull(wrist);
  }

  /**
   * Copies this arm state with a new shoulder rotation.
   *
   * @param newShoulder the new shoulder rotation.
   * @return a copy of this arm state with a new shoulder rotation.
   */
  public ArmState withShoulder(Rotation2d newShoulder) {
    return new ArmState(newShoulder, elbow, wrist);
  }

  /**
   * Copies this arm state with a new elbow rotation.
   *
   * @param newElbow the new elbow rotation.
   * @return a copy of this arm state with a new elbow rotation.
   */
  public ArmState withElbow(Rotation2d newElbow) {
    return new ArmState(shoulder, newElbow, wrist);
  }

  /**
   * Copies this arm state with a new wrist rotation.
   *
   * @param newWrist the new wrist rotation.
   * @return a copy of this arm state with a new wrist rotation.
   */
  public ArmState withWrist(Rotation2d newWrist) {
    return new ArmState(shoulder, elbow, newWrist);
  }
}
