package frc.robot.superstructure;

import frc.robot.arm.ArmState;
import frc.robot.intake.IntakeState;
import frc.robot.shooter.ShooterState;
import java.util.Objects;

/** Superstructure state. */
public record SuperstructureState(
    ArmState armState, IntakeState intakeState, ShooterState shooterState) {

  /** Stowed state. */
  public static final SuperstructureState STOWED =
      new SuperstructureState(ArmState.STOWED, IntakeState.IDLE, ShooterState.IDLING);

  /** Intaking state. */
  public static final SuperstructureState INTAKE =
      new SuperstructureState(ArmState.STOWED, IntakeState.INTAKE, ShooterState.INTAKE);

  /** Ready ejecting state. */
  public static final SuperstructureState EJECT_POSITION =
      new SuperstructureState(ArmState.EJECT, IntakeState.EJECTING, ShooterState.IDLING);

  /** Ejecting state. */
  public static final SuperstructureState EJECT =
      new SuperstructureState(ArmState.EJECT, IntakeState.EJECTING, ShooterState.EJECT);

  /** Subwoofer shooting state. */
  public static final SuperstructureState SUBWOOFER =
      new SuperstructureState(ArmState.SUBWOOFER, IntakeState.IDLE, ShooterState.SUBWOOFER);

  /** Skim shooting state. */
  public static final SuperstructureState SKIM =
      new SuperstructureState(ArmState.SKIM, IntakeState.IDLE, ShooterState.SKIM);

  /** Amp shooting state. */
  public static final SuperstructureState AMP =
      new SuperstructureState(ArmState.AMP, IntakeState.IDLE, ShooterState.AMP);

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
