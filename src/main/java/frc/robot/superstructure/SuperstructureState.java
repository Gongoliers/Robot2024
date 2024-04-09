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
    ArmState armState,
    IntakeState intakeState,
    ShooterState shooterState) {

  public static final SuperstructureState INITIAL =
      new SuperstructureState(ArmState.INITIAL, IntakeState.IDLE, ShooterState.IDLE);

  public static final SuperstructureState STOW =
      new SuperstructureState(ArmState.STOW, IntakeState.IDLE, ShooterState.IDLE);

  public static final SuperstructureState INTAKE =
      new SuperstructureState(
          ArmState.STOW,
          IntakeState.INTAKE,
          ShooterState.INTAKE);

  public static final SuperstructureState PULL = new SuperstructureState(ArmState.STOW, IntakeState.IDLE, ShooterState.PULL);

  public static final SuperstructureState EJECT = new SuperstructureState(ArmState.EJECT, IntakeState.IDLE, ShooterState.EJECT);

  public static final SuperstructureState SPEAKER =
      new SuperstructureState(
          ArmState.SHOOT,
          IntakeState.IDLE,
          ShooterState.SPEAKER);

  public static final SuperstructureState PASS =
      new SuperstructureState(
          ArmState.SHOOT,
          IntakeState.IDLE,
          ShooterState.PASS);

  public static final SuperstructureState CLIMB =
      new SuperstructureState(ArmState.CLIMB, IntakeState.IDLE, ShooterState.IDLE);

  public static final SuperstructureState AMP =
      new SuperstructureState(
          ArmState.AMP, IntakeState.IDLE, ShooterState.AMP);

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
