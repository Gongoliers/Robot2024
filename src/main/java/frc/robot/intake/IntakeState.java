package frc.robot.intake;

import edu.wpi.first.math.MathUtil;
import frc.robot.intake.IntakeConstants.BackRollerConstants;
import frc.robot.intake.IntakeConstants.FrontRollerConstants;

import java.util.Objects;

public record IntakeState(
    double frontRollerVelocityRotationsPerSecond, double backRollerVelocityRotationsPerSecond) {

  public static final IntakeState IDLE = new IntakeState(0, 0);

  public static final IntakeState INTAKE = new IntakeState(FrontRollerConstants.INTAKE_SPEED, BackRollerConstants.INTAKE_SPEED);

  public static final IntakeState EJECT = new IntakeState(FrontRollerConstants.EJECT_SPEED, BackRollerConstants.EJECT_SPEED);

  public IntakeState {
    Objects.requireNonNull(frontRollerVelocityRotationsPerSecond);
    Objects.requireNonNull(backRollerVelocityRotationsPerSecond);
  }

  public boolean at(IntakeState other) {
    final double kToleranceRotationsPerSecond = 1;

    return MathUtil.isNear(
            frontRollerVelocityRotationsPerSecond,
            other.frontRollerVelocityRotationsPerSecond,
            kToleranceRotationsPerSecond)
        && MathUtil.isNear(
            backRollerVelocityRotationsPerSecond,
            other.backRollerVelocityRotationsPerSecond,
            kToleranceRotationsPerSecond);
  }
}
