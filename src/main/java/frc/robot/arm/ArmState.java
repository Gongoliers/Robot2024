package frc.robot.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import frc.robot.arm.ArmConstants.ShoulderConstants;
import java.util.Objects;

/** Represents an arm's state */
public record ArmState(State shoulderRotations) {

  /** State for stow position. */
  public static final ArmState STOW_POSITION = new ArmState(ShoulderConstants.STOW_ANGLE);

  /** State for flat position. */
  public static final ArmState FLAT_POSITION = new ArmState(ShoulderConstants.FLAT_ANGLE);

  /** State for amp position. */
  public static final ArmState AMP_POSITION = new ArmState(Rotation2d.fromDegrees(60));

  public ArmState {
    Objects.requireNonNull(shoulderRotations);
  }

  public ArmState(Rotation2d shoulder) {
    this(new State(shoulder.getRotations(), 0));
  }

  public boolean at(ArmState other) {
    return MathUtil.isNear(
        shoulderRotations.position,
        other.shoulderRotations.position,
        Units.degreesToRotations(2.0));
  }
}
