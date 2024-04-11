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
      new SuperstructureState(ArmState.EJECT, IntakeState.EJECT, ShooterState.IDLE);

  public static final SuperstructureState EJECT =
      new SuperstructureState(ArmState.EJECT, IntakeState.EJECT, ShooterState.EJECT);

  public static final SuperstructureState SUBWOOFER_PULL =
      new SuperstructureState(ArmState.SUBWOOFER, IntakeState.IDLE, ShooterState.PULL);

  public static final SuperstructureState SUBWOOFER_READY =
      new SuperstructureState(ArmState.SUBWOOFER, IntakeState.IDLE, ShooterState.SPEAKER_READY);

  public static final SuperstructureState SUBWOOFER_SHOOT =
      new SuperstructureState(ArmState.SUBWOOFER, IntakeState.IDLE, ShooterState.SPEAKER_SHOOT);

  public static final SuperstructureState PODIUM_PULL =
      new SuperstructureState(ArmState.PODIUM, IntakeState.IDLE, ShooterState.PULL);

  public static final SuperstructureState PODIUM_READY =
      new SuperstructureState(ArmState.PODIUM, IntakeState.IDLE, ShooterState.PODIUM_READY);

  public static final SuperstructureState PODIUM_SHOOT =
      new SuperstructureState(ArmState.PODIUM, IntakeState.IDLE, ShooterState.PODIUM_SHOOT);

  public static final SuperstructureState LOB_PULL =
      new SuperstructureState(ArmState.LOB, IntakeState.IDLE, ShooterState.PULL);

  public static final SuperstructureState LOB_READY =
      new SuperstructureState(ArmState.LOB, IntakeState.IDLE, ShooterState.LOB_READY);

  public static final SuperstructureState LOB_SHOOT =
      new SuperstructureState(ArmState.LOB, IntakeState.IDLE, ShooterState.LOB_SHOOT);

  public static final SuperstructureState SKIM_PULL =
      new SuperstructureState(ArmState.SKIM, IntakeState.IDLE, ShooterState.PULL);

  public static final SuperstructureState SKIM_READY =
      new SuperstructureState(ArmState.SKIM, IntakeState.IDLE, ShooterState.SKIM_READY);

  public static final SuperstructureState SKIM_SHOOT =
      new SuperstructureState(ArmState.SKIM, IntakeState.IDLE, ShooterState.SKIM_SHOOT);

  public static final SuperstructureState AMP_PULL =
      new SuperstructureState(ArmState.AMP, IntakeState.IDLE, ShooterState.PULL);

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
