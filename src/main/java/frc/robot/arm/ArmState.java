package frc.robot.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import java.util.Objects;

/** Arm state. */
public record ArmState(State shoulderRotations) {

  /** Stow position. */
  public static final ArmState STOW_POSITION = new ArmState(Rotation2d.fromDegrees(-26));

  /** Subwoofer shot position. */
  public static final ArmState SUBWOOFER_POSITION = new ArmState(Rotation2d.fromDegrees(-26));

  /** Eject position. */
  public static final ArmState EJECT_POSITION = new ArmState(Rotation2d.fromDegrees(30));

  /** Skim shot position. */
  public static final ArmState SKIM_POSITION = new ArmState(Rotation2d.fromDegrees(30));

  /** Amp position. */
  public static final ArmState AMP_POSITION = new ArmState(Rotation2d.fromDegrees(60));

  /**
   * Arm state.
   *
   * @param shoulderRotations the shoulder state in rotations and rotations per second.
   */
  public ArmState {
    Objects.requireNonNull(shoulderRotations);
  }

  /**
   * Arm state.
   *
   * @param shoulder the shoulder position.
   */
  public ArmState(Rotation2d shoulder) {
    this(new State(shoulder.getRotations(), 0));
  }

  /**
   * Returns true if this arm state is at another arm state.
   *
   * @param other another arm state.
   * @return true if this arm state is at another arm state.
   */
  public boolean at(ArmState other) {
    return MathUtil.isNear(
        shoulderRotations.position,
        other.shoulderRotations.position,
        Units.degreesToRotations(2.0));
  }
}
