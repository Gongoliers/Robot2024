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

/** Subsystem class for the superstructure subsystem. */
public class Superstructure extends Subsystem {

  /** Instance variable for the superstructure subsystem singleton. */
  private static Superstructure instance = null;

  /** Reference to the arm subsystem. */
  private final Arm arm;

  /** Reference to the intake subsystem. */
  private final Intake intake;

  /** Reference to the shooter subsystem. */
  private final Shooter shooter;

  private SuperstructureState measurement, goal;

  /** Creates a new instance of the superstructure subsystem. */
  private Superstructure() {
    arm = Arm.getInstance();
    intake = Intake.getInstance();
    shooter = Shooter.getInstance();

    setPosition(SuperstructureState.STOW);

    goal = SuperstructureState.STOW;
  }

  /**
   * Gets the instance of the superstructure subsystem.
   *
   * @return the instance of the superstructre subsystem.
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

    SuperstructureMechanism.getInstance().updateSuperstructure(measurement);
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

  public void setPosition(SuperstructureState state) {
    arm.setPosition(state.armState());
  }

  public void setGoal(SuperstructureState goal) {
    this.goal = goal;

    arm.setGoal(goal.armState());
    intake.setGoal(goal.intakeState());
    shooter.setGoal(goal.shooterState());
  }

  public boolean atGoal() {
    return arm.atGoal() && intake.atGoal() && shooter.atGoal();
  }

  private Command instant(SuperstructureState goal) {
    return runOnce(() -> setGoal(goal));
  }

  private Command hold(SuperstructureState goal) {
    return run(() -> setGoal(goal));
  }

  private Command to(SuperstructureState goal) {
    return run(() -> setGoal(goal)).until(this::atGoal);
  }

  public Command stow() {
    return hold(SuperstructureState.STOW).withName("STOW");
  }

  public Command intake() {
    return to(SuperstructureState.STOW)
        .andThen(hold(SuperstructureState.INTAKE))
        .withName("INTAKE");
  }

  public Command intakeInstant() {
    return instant(SuperstructureState.INTAKE).withName("INTAKE_IMMEDIATE");
  }

  private Command pull(SuperstructureState shot) {
    final SuperstructureState pull =
        new SuperstructureState(shot.armState(), IntakeState.IDLE, ShooterState.PULL);

    return hold(pull).withTimeout(SuperstructureConstants.PULL_DURATION);
  }

  private Command ready(SuperstructureState shot) {
    final ShooterState spin =
        new ShooterState(shot.shooterState().flywheelVelocityRotationsPerSecond(), 0);

    final SuperstructureState ready =
        new SuperstructureState(shot.armState(), IntakeState.IDLE, spin);

    return to(ready).withTimeout(SuperstructureConstants.READY_DURATION);
  }

  private Command shoot(SuperstructureState shot) {
    return pull(shot)
        .andThen(ready(shot))
        .andThen(Commands.waitSeconds(SuperstructureConstants.READY_PAUSE_DURATION))
        .andThen(hold(shot));
  }

  private Command shootNoPull(SuperstructureState shot) {
    return ready(shot).andThen(hold(shot));
  }
  
  public Command subwooferNoPull() {
    return shootNoPull(SuperstructureState.SUBWOOFER).withName("SUBWOOFER_NO_PULL");
  } 

  public Command subwoofer() {
    return shoot(SuperstructureState.SUBWOOFER).withName("SUBWOOFER");
  }

  public Command podium() {
    return shoot(SuperstructureState.PODIUM).withName("PODIUM");
  }

  public Command lob() {
    return shoot(SuperstructureState.LOB).withName("LOB");
  }

  public Command skim() {
    return shoot(SuperstructureState.SKIM).withName("SKIM");
  }

  public Command amp() {
    return shoot(SuperstructureState.AMP).withName("AMP");
  }

  public Command bloop() {
    return shoot(SuperstructureState.BLOOP).withName("BLOOP");
  }

  public Command eject() {
    return to(SuperstructureState.EJECT_POSITION)
        .andThen(Commands.waitSeconds(SuperstructureConstants.EJECT_PAUSE_DURATION))
        .andThen(hold(SuperstructureState.EJECT))
        .withName("EJECT");
  }
}
