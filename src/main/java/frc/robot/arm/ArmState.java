package frc.robot.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import java.util.Objects;

public record ArmState(State shoulderRotations) {

  public static final ArmState STOW = new ArmState(Rotation2d.fromDegrees(-26));

  public static final ArmState SUBWOOFER = new ArmState(Rotation2d.fromDegrees(-26));

  public static final ArmState EJECT = new ArmState(Rotation2d.fromDegrees(30));

  public static final ArmState SKIM = new ArmState(Rotation2d.fromDegrees(30));

  public static final ArmState LOB = new ArmState(Rotation2d.fromDegrees(-26));

  public static final ArmState AMP = new ArmState(Rotation2d.fromDegrees(60));

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
