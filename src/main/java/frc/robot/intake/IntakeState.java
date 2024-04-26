package frc.robot.intake;

import edu.wpi.first.math.MathUtil;
import java.util.Objects;

public record IntakeState(
    double frontRollerVelocityRotationsPerSecond, double backRollerVelocityRotationsPerSecond) {

  public static final IntakeState IDLE = new IntakeState(0, 0);

  public static final IntakeState INTAKE = new IntakeState((double) 34, (double) 34);

  public static final IntakeState EJECT = new IntakeState((double) -34, (double) -34);

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
