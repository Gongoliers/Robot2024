package frc.robot.superstructure;

import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.robot.arm.Arm;
import frc.robot.arm.ArmState;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeState;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.ShooterState;

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
  }

  private void addStateToShuffleboard(ShuffleboardTab tab, String name, Supplier<SuperstructureState> state) {
    ShuffleboardLayout layout = Telemetry.addColumn(tab, name);

    layout.addDouble(
        "Shoulder Position (deg)",
        () -> Units.rotationsToDegrees(state.get().armState().shoulderRotations().position));
    layout.addDouble(
        "Shoulder Velocity (dps)",
        () -> Units.rotationsToDegrees(state.get().armState().shoulderRotations().velocity));

    layout.addDouble(
        "Front Roller Velocity (rps)", () -> state.get().intakeState().frontRollerVelocityRotationsPerSecond());

    layout.addDouble(
        "Back Roller Velocity (rps)", () -> state.get().intakeState().backRollerVelocityRotationsPerSecond());

    layout.addDouble(
        "Flywheel Velocity (rps)", () -> state.get().shooterState().flywheelVelocityRotationsPerSecond());

    layout.addDouble(
        "Serializer Velocity (rps)", () -> state.get().shooterState().serializerVelocityRotationsPerSecond());
  }

  private void updateMeasurement() {
    ArmState measuredShoulderState = arm.getState();

    IntakeState measuredIntakeState = intake.getState();

    ShooterState measuredShooterState = shooter.getState();

    measurement =
        new SuperstructureState(
            measuredShoulderState,
            measuredIntakeState,
            measuredShooterState);

    SuperstructureMechanism.getInstance().updateSuperstructure(measurement);
  }

  public SuperstructureState getState() {
    updateMeasurement();

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

  private Command to(SuperstructureState goal) {
    return new ToGoal(goal);
  }

  public Command stow() {
    return to(SuperstructureState.STOW);
  }

  public Command intake() {
    return to(SuperstructureState.INTAKE);
  }

  public Command pull() {
    return Commands.deadline(Commands.waitSeconds(0.15), to(SuperstructureState.PULL));
  }

  public Command shoot() {
    return pull().andThen(to(SuperstructureState.SPEAKER_SHOOT));
  }

  public Command pass() {
    return pull().andThen(to(SuperstructureState.PASS_SPIN).andThen(to(SuperstructureState.PASS_SHOOT)));
  }

  public Command ampPosition() {
    return to(SuperstructureState.AMP_POSITION);
  }

  public Command ampShoot() {
    return ampPosition().andThen(to(SuperstructureState.AMP_SHOOT));
  }
  
  public Command eject() {
    return to(SuperstructureState.EJECT_POSITION).andThen(to(SuperstructureState.EJECT));
  }
}
