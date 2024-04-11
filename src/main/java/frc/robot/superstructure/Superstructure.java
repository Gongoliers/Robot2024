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
import frc.robot.shooter.Shooter;
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

    setPosition(SuperstructureState.INITIAL);

    goal = SuperstructureState.INITIAL;
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
    updateMeasurement();
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    addStateToShuffleboard(tab, "Measurement", () -> measurement);
    addStateToShuffleboard(tab, "Goal", () -> goal);

    ShuffleboardLayout state = Telemetry.addColumn(tab, "State");

    state.addString(
        "State",
        () -> this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "NONE");

    ShuffleboardLayout at = Telemetry.addColumn(tab, "At Goal?");

    at.addBoolean("At Goal?", () -> at(goal));
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

  private void updateMeasurement() {
    measurement = new SuperstructureState(arm.getState(), intake.getState(), shooter.getState());

    SuperstructureMechanism.getInstance().updateSuperstructure(measurement);
  }

  public SuperstructureState getState() {
    return measurement;
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

  public boolean at(SuperstructureState goal) {
    updateMeasurement();

    return measurement.atGoal(goal);
  }

  private Command hold(SuperstructureState goal) {
    return run(() -> setGoal(goal));
  }

  private Command to(SuperstructureState goal) {
    return run(() -> setGoal(goal)).until(() -> at(goal));
  }

  public Command stow() {
    return hold(SuperstructureState.STOW).withName("STOW");
  }

  public Command intake() {
    return to(SuperstructureState.STOW)
        .andThen(hold(SuperstructureState.INTAKE))
        .withName("INTAKE");
  }

  public Command subwoofer() {
    return hold(SuperstructureState.SUBWOOFER_PULL)
        .withTimeout(SuperstructureConstants.PULL_DURATION)
        .andThen(to(SuperstructureState.SUBWOOFER_READY))
        .andThen(hold(SuperstructureState.SUBWOOFER_SHOOT))
        .withName("SPEAKER");
  }

  public Command pass() {
    return hold(SuperstructureState.PASS_PULL)
        .withTimeout(SuperstructureConstants.PULL_DURATION)
        .andThen(to(SuperstructureState.PASS_READY))
        .andThen(hold(SuperstructureState.PASS_SHOOT))
        .withName("PASS");
  }

  public Command amp() {
    return hold(SuperstructureState.AMP_PULL)
        .withTimeout(SuperstructureConstants.PULL_DURATION)
        .andThen(to(SuperstructureState.AMP_POSITION))
        .andThen(hold(SuperstructureState.AMP_SHOOT))
        .withName("AMP");
  }

  public Command eject() {
    return to(SuperstructureState.EJECT_POSITION)
        .andThen(Commands.waitSeconds(SuperstructureConstants.EJECT_PAUSE))
        .andThen(hold(SuperstructureState.EJECT))
        .withName("EJECT");
  }
}
