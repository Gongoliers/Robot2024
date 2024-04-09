package frc.robot.intake;

import java.util.Objects;

import edu.wpi.first.math.MathUtil;
import frc.robot.intake.IntakeConstants.RollerConstants;

public record IntakeState(double frontRollerVelocityRotationsPerSecond, double backRollerVelocityRotationsPerSecond) {
   
    public IntakeState {
        Objects.requireNonNull(frontRollerVelocityRotationsPerSecond);
        Objects.requireNonNull(backRollerVelocityRotationsPerSecond);
    }

  public boolean at(IntakeState other) {
    return MathUtil.isNear(
        frontRollerVelocityRotationsPerSecond,
        other.frontRollerVelocityRotationsPerSecond,
        RollerConstants.SPEED_TOLERANCE)
    && MathUtil.isNear(
        backRollerVelocityRotationsPerSecond,
        other.backRollerVelocityRotationsPerSecond,
        RollerConstants.SPEED_TOLERANCE);
  }

}
