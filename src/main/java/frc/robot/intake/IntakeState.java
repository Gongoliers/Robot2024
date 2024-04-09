package frc.robot.intake;

import java.util.Objects;

import edu.wpi.first.math.MathUtil;

public record IntakeState(double frontRollerVelocityRotationsPerSecond, double backRollerVelocityRotationsPerSecond) {

  public static final IntakeState IDLE = new IntakeState(0, 0);

  public static final IntakeState INTAKE = new IntakeState(34, 34);
   
    public IntakeState {
        Objects.requireNonNull(frontRollerVelocityRotationsPerSecond);
        Objects.requireNonNull(backRollerVelocityRotationsPerSecond);
    }

  public boolean at(IntakeState other) {
    final double kToleranceRotationsPerSecond = 2.5;

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
