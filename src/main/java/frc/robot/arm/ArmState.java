package frc.robot.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import frc.robot.arm.ArmConstants.ShoulderConstants;
import java.util.Objects;

public record ArmState(State shoulderRotations) {

  public static final ArmState INITIAL = new ArmState(ShoulderConstants.STOW);

  public static final ArmState STOW = new ArmState(ShoulderConstants.STOW);

  public static final ArmState SUBWOOFER = new ArmState(ShoulderConstants.SUBWOOFER);

  public static final ArmState PODIUM = new ArmState(ShoulderConstants.PODIUM);

  public static final ArmState EJECT = new ArmState(ShoulderConstants.EJECT);

  public static final ArmState SKIM = new ArmState(ShoulderConstants.SKIM);

  public static final ArmState LOB = new ArmState(ShoulderConstants.LOB);

  public static final ArmState AMP = new ArmState(ShoulderConstants.AMP);

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
