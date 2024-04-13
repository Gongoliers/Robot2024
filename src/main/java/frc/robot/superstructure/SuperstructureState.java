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

  public static final SuperstructureState STOW =
      new SuperstructureState(ArmState.STOW, IntakeState.IDLE, ShooterState.IDLE);

  public static final SuperstructureState INTAKE =
      new SuperstructureState(ArmState.STOW, IntakeState.INTAKE, ShooterState.INTAKE);

  public static final SuperstructureState EJECT_POSITION =
      new SuperstructureState(ArmState.EJECT, IntakeState.EJECT, ShooterState.IDLE);

  public static final SuperstructureState EJECT =
      new SuperstructureState(ArmState.EJECT, IntakeState.EJECT, ShooterState.EJECT);

  public static final SuperstructureState SUBWOOFER =
      new SuperstructureState(ArmState.SUBWOOFER, IntakeState.IDLE, ShooterState.SUBWOOFER);

  public static final SuperstructureState PODIUM =
      new SuperstructureState(ArmState.PODIUM, IntakeState.IDLE, ShooterState.PODIUM);

  public static final SuperstructureState LOB =
      new SuperstructureState(ArmState.LOB, IntakeState.IDLE, ShooterState.LOB);

  public static final SuperstructureState SKIM =
      new SuperstructureState(ArmState.SKIM, IntakeState.IDLE, ShooterState.SKIM);

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
