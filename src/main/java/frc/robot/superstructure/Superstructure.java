package frc.robot.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.robot.arm.Arm;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeState;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.ShooterState;
import java.util.function.Supplier;

/** Superstructure subsystem. */
public class Superstructure extends Subsystem {

  /** Superstructure singleton. */
  private static Superstructure instance = null;

  /** Arm subsystem reference. */
  private final Arm arm;

  /** Intake subsystem reference. */
  private final Intake intake;

  /** Shooter subsystem reference. */
  private final Shooter shooter;

  /** Superstructure measurement. Updated periodically. */
  private SuperstructureState measurement;

  /** Superstructure goal. */
  private SuperstructureState goal;

  /** Initializes the superstructure subsystem. */
  private Superstructure() {
    arm = Arm.getInstance();
    intake = Intake.getInstance();
    shooter = Shooter.getInstance();

    arm.setPosition(SuperstructureState.STOWED.armState());

    goal = SuperstructureState.STOWED;
  }

  /**
   * Returns the superstructure subsystem instance.
   *
   * @return the superstructure subsystem instance.
   */
  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
    }

    return instance;
  }

  @Override
  public void periodic() {
    measurement = new SuperstructureState(arm.getState(), intake.getState(), shooter.getState());

    SuperstructureMechanism.getInstance().update(measurement);
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    addStateToShuffleboard(tab, "Measurement", () -> measurement);
    addStateToShuffleboard(tab, "Goal", () -> goal);

    Telemetry.addColumn(tab, "State")
        .addString(
            "State",
            () -> this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "NONE");

    Telemetry.addColumn(tab, "At Goal?").addBoolean("At Goal?", this::atGoal);
  }

  /**
   * Adds a superstructure state to Shuffleboard.
   *
   * @param tab the Shuffleboard tab.
   * @param name the name of the superstructure state.
   * @param state the state to display on Shuffleboard.
   */
  private void addStateToShuffleboard(
      ShuffleboardTab tab, String name, Supplier<SuperstructureState> state) {
    ShuffleboardLayout layout = Telemetry.addColumn(tab, name);

    layout.addDouble(
        "Shoulder Position (deg)",
        () -> Units.rotationsToDegrees(state.get().armState().shoulderRotations().position));
    layout.addDouble(
        "Shoulder Velocity (dps)",
        () -> Units.rotationsToDegrees(state.get().armState().shoulderRotations().velocity));

    layout.addDouble(
        "Front Roller Velocity (rps)",
        () -> state.get().intakeState().frontRollerVelocityRotationsPerSecond());

    layout.addDouble(
        "Back Roller Velocity (rps)",
        () -> state.get().intakeState().backRollerVelocityRotationsPerSecond());

    layout.addDouble(
        "Flywheel Velocity (rps)",
        () -> state.get().shooterState().flywheelVelocityRotationsPerSecond());

    layout.addDouble(
        "Serializer Velocity (rps)",
        () -> state.get().shooterState().serializerVelocityRotationsPerSecond());
  }

  /**
   * Sets the superstructure goal.
   *
   * @param goal the superstructure goal.
   */
  public void setGoal(SuperstructureState goal) {
    this.goal = goal;

    arm.setGoal(goal.armState());
    intake.setGoal(goal.intakeState());
    shooter.setGoal(goal.shooterState());
  }

  /**
   * Returns true if the superstructure is at the goal state.
   *
   * @return true if the superstructure is at the goal state.
   */
  public boolean atGoal() {
    return arm.atGoal() && intake.atGoal() && shooter.atGoal();
  }

  /**
   * Sets the superstructure goal and ends immediately.
   *
   * @param goal the superstructure goal.
   * @return a command that sets the superstructure goal and ends immediately.
   */
  private Command instant(SuperstructureState goal) {
    return runOnce(() -> setGoal(goal));
  }

  /**
   * Sets the superstructure goal and ends when the goal is reached.
   *
   * @param goal the superstructure goal.
   * @return a command that sets the superstructure goal and ends when the goal is reached.
   */
  private Command to(SuperstructureState goal) {
    return run(() -> setGoal(goal)).until(this::atGoal);
  }

  /**
   * Sets the superstructure goal and ends when interrupted.
   *
   * @param goal the superstructure goal.
   * @return a command that sets the superstructure goal and ends when interrupted.
   */
  private Command hold(SuperstructureState goal) {
    return run(() -> setGoal(goal));
  }

  /**
   * Stows the superstructure. Ends when interrupted.
   *
   * @return a command that stows the superstructure.
   */
  public Command stow() {
    return hold(SuperstructureState.STOWED).withName("STOWED");
  }

  /**
   * Stows then intakes. Ends when interrupted.
   *
   * @return a command that stows then intakes.
   */
  public Command intake() {
    return to(SuperstructureState.STOWED)
        .andThen(hold(SuperstructureState.INTAKE))
        .withName("INTAKE");
  }

  /**
   * Intakes. Ends immediately.
   *
   * @return a command that intakes.
   */
  public Command intakeInstant() {
    return instant(SuperstructureState.INTAKE).withName("INTAKE_IMMEDIATE");
  }

  /**
   * Pulls a note while moving to the shot position. Ends after a duration is elapsed.
   *
   * @param shot the shot position to move to.
   * @return a command that pulls a note while moving to the shot position.
   */
  private Command pull(SuperstructureState shot) {
    final SuperstructureState pull =
        new SuperstructureState(shot.armState(), IntakeState.IDLE, ShooterState.PULL);

    return hold(pull).withTimeout(0.2);
  }

  /**
   * Spins up and moves to a shot but does not fire it. Ends after the shot is spun up and in
   * position or a duration is elapsed.
   *
   * @param shot the shot to spin up for.
   * @return a commands that spins up and moves to a shot but does not fire it.
   */
  private Command ready(SuperstructureState shot) {
    final ShooterState spin =
        new ShooterState(shot.shooterState().flywheelVelocityRotationsPerSecond(), 0);

    final SuperstructureState ready =
        new SuperstructureState(shot.armState(), IntakeState.IDLE, spin);

    return to(ready).withTimeout(1.0);
  }

  /**
   * Prepares a shot. Ends after the shot is spun up and in position or a duration is elapsed.
   *
   * @param shot the shot.
   * @return a command that prepares a shot.
   */
  public Command prepare(SuperstructureState shot) {
    return pull(shot).andThen(ready(shot));
  }

  /**
   * Readies and shoots a shot. Ends when interrupted.
   *
   * @param shot the shot.
   * @return a command that readies and shoots a shot.
   */
  public Command shoot(SuperstructureState shot) {
    return ready(shot).andThen(Commands.waitSeconds(0.25)).andThen(hold(shot));
  }

  /**
   * Pulls, readies, and shoots a shot. Ends when interrupted.
   *
   * @param shot the shot.
   * @return a command that pulls, readies, and shoots a shot.
   */
  private Command autoShoot(SuperstructureState shot) {
    return pull(shot).andThen(shoot(shot));
  }

  /**
   * Automatically shoots a note into the speaker from the subwoofer. Ends when interrupted.
   *
   * @return a command that automatically shoots a note into the speaker from the subwoofer.
   */
  public Command subwoofer() {
    return autoShoot(SuperstructureState.SUBWOOFER).withName("SUBWOOFER");
  }

  /**
   * Automatically skims a note across the floor. Ends when interrupted.
   *
   * @return a command that automatically skims a note across the floor.
   */
  public Command skim() {
    return autoShoot(SuperstructureState.SKIM).withName("SKIM");
  }

  /**
   * Automatically drops a note into the amp. Ends when interrupted.
   *
   * @return a command that drops a note into the amp.
   */
  public Command amp() {
    return autoShoot(SuperstructureState.AMP).withName("AMP");
  }

  /**
   * Ejects a note. Ends when interrupted.
   *
   * @return a command that ejects a note.
   */
  public Command eject() {
    return to(SuperstructureState.EJECT_POSITION)
        .andThen(Commands.waitSeconds(0.25))
        .andThen(hold(SuperstructureState.EJECT))
        .withName("EJECTING");
  }
}
