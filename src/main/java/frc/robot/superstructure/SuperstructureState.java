package frc.robot.superstructure;

import frc.robot.arm.ArmState;
import frc.robot.intake.IntakeState;
import frc.robot.shooter.ShooterState;
import java.util.Objects;

/** Represents the state of the superstructure. */
public record SuperstructureState(
    ArmState armState, IntakeState intakeState, ShooterState shooterState) {

  public static final SuperstructureState STOW =
      new SuperstructureState(ArmState.STOW_POSITION, IntakeState.IDLE, ShooterState.IDLE);

  public static final SuperstructureState INTAKE =
      new SuperstructureState(ArmState.STOW_POSITION, IntakeState.INTAKE, ShooterState.INTAKE);

  public static final SuperstructureState EJECT_POSITION =
      new SuperstructureState(ArmState.FLAT_POSITION, IntakeState.EJECT, ShooterState.IDLE);

  public static final SuperstructureState EJECT =
      new SuperstructureState(ArmState.FLAT_POSITION, IntakeState.EJECT, ShooterState.EJECT);

  public static final SuperstructureState SUBWOOFER =
      new SuperstructureState(ArmState.STOW_POSITION, IntakeState.IDLE, ShooterState.SUBWOOFER);

  public static final SuperstructureState SKIM =
      new SuperstructureState(ArmState.FLAT_POSITION, IntakeState.IDLE, ShooterState.SKIM);

  public static final SuperstructureState AMP =
      new SuperstructureState(ArmState.AMP_POSITION, IntakeState.IDLE, ShooterState.AMP);

  public static final SuperstructureState BLOOP =
      new SuperstructureState(ArmState.STOW_POSITION, IntakeState.IDLE, ShooterState.BLOOP);

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
}
