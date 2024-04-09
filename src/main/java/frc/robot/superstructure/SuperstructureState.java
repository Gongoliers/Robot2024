package frc.robot.superstructure;

import frc.robot.arm.Arm;
import frc.robot.arm.ArmState;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeState;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.ShooterState;
import java.util.Objects;

/** Represents the state of the superstructure. */
public record SuperstructureState(
    ArmState armState, IntakeState intakeState, ShooterState shooterState) {

  public static final SuperstructureState INITIAL =
      new SuperstructureState(ArmState.INITIAL, IntakeState.IDLE, ShooterState.IDLE);

  public static final SuperstructureState STOW =
      new SuperstructureState(ArmState.STOW, IntakeState.IDLE, ShooterState.IDLE);

  public static final SuperstructureState INTAKE =
      new SuperstructureState(ArmState.STOW, IntakeState.INTAKE, ShooterState.INTAKE);

  public static final SuperstructureState EJECT_POSITION =
      new SuperstructureState(ArmState.EJECT, IntakeState.IDLE, ShooterState.IDLE);

  public static final SuperstructureState EJECT =
      new SuperstructureState(ArmState.EJECT, IntakeState.IDLE, ShooterState.EJECT);

  public static final SuperstructureState SPEAKER_PULL =
      new SuperstructureState(ArmState.SPEAKER, IntakeState.IDLE, ShooterState.PULL);

  public static final SuperstructureState SPEAKER_READY =
      new SuperstructureState(ArmState.SPEAKER, IntakeState.IDLE, ShooterState.SPEAKER_READY);

  public static final SuperstructureState SPEAKER_SHOOT =
      new SuperstructureState(ArmState.SPEAKER, IntakeState.IDLE, ShooterState.SPEAKER_SHOOT);

  public static final SuperstructureState PASS_PULL =
      new SuperstructureState(ArmState.PASS, IntakeState.IDLE, ShooterState.PULL);

  public static final SuperstructureState PASS_READY =
      new SuperstructureState(ArmState.PASS, IntakeState.IDLE, ShooterState.PASS_READY);

  public static final SuperstructureState PASS_SHOOT =
      new SuperstructureState(ArmState.PASS, IntakeState.IDLE, ShooterState.PASS_SHOOT);

  public static final SuperstructureState CLIMB =
      new SuperstructureState(ArmState.CLIMB, IntakeState.IDLE, ShooterState.IDLE);

  public static final SuperstructureState AMP_POSITION =
      new SuperstructureState(ArmState.AMP, IntakeState.IDLE, ShooterState.IDLE);

  public static final SuperstructureState AMP_SHOOT =
      new SuperstructureState(ArmState.AMP, IntakeState.IDLE, ShooterState.AMP_SHOOT);

  /**
   * Creates a new superstructure state.
   *
   * @param armState
   * @param intakeState
   * @param shooterState
   */
  public SuperstructureState {
    Objects.requireNonNull(armState);
    Objects.requireNonNull(intakeState);
    Objects.requireNonNull(shooterState);
  }

  /**
   * Returns true if at the superstructure goal.
   *
   * @param goal
   * @return true if at the superstructure goal.
   */
  public boolean atGoal(SuperstructureState goal) {
    return Arm.getInstance().atGoal()
        && Intake.getInstance().atGoal()
        && Shooter.getInstance().atGoal();
  }
}
