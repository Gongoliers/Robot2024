package frc.robot.arm;

import java.util.Objects;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.arm.ArmConstants.ShoulderConstants;

public record ArmState(State shoulderRotations) {
    
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
        ShoulderConstants.TOLERANCE.getRotations());
  }

}
