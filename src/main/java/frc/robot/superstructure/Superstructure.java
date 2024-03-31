package frc.robot.superstructure;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.robot.arm.Arm;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeConstants.RollerConstants;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.ShooterConstants.FlywheelConstants;
import frc.robot.shooter.ShooterConstants.SerializerConstants;

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

  private SuperstructureState measurement, setpoint, goal;

  /** Creates a new instance of the superstructure subsystem. */
  private Superstructure() {
    arm = Arm.getInstance();
    intake = Intake.getInstance();
    shooter = Shooter.getInstance();

    setPosition(SuperstructureState.INITIAL);

    setpoint = SuperstructureState.INITIAL;
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
    updateSetpoint();
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    ShuffleboardLayout measurement = Telemetry.addColumn(tab, "Measurement");

    measurement.addDouble(
        "Shoulder Position (deg)",
        () -> Units.rotationsToDegrees(this.measurement.shoulderAngleRotations().position));
    measurement.addDouble(
        "Shoulder Velocity (dps)",
        () -> Units.rotationsToDegrees(this.measurement.shoulderAngleRotations().velocity));

    measurement.addDouble(
        "Roller Velocity (rps)", () -> this.measurement.rollerVelocityRotationsPerSecond());

    measurement.addDouble(
        "Flywheel Velocity (rps)", () -> this.measurement.flywheelVelocityRotationsPerSecond());

    measurement.addDouble(
        "Serializer Velocity (rps)", () -> this.measurement.serializerVelocityRotationsPerSecond());

    ShuffleboardLayout setpoint = Telemetry.addColumn(tab, "Setpoint");

    setpoint.addDouble(
        "Shoulder Position (deg)",
        () -> Units.rotationsToDegrees(this.setpoint.shoulderAngleRotations().position));
    setpoint.addDouble(
        "Shoulder Velocity (dps)",
        () -> Units.rotationsToDegrees(this.setpoint.shoulderAngleRotations().velocity));

    setpoint.addDouble(
        "Roller Velocity (rps)", () -> this.setpoint.rollerVelocityRotationsPerSecond());

    setpoint.addDouble(
        "Flywheel Velocity (rps)", () -> this.setpoint.flywheelVelocityRotationsPerSecond());

    setpoint.addDouble(
        "Serializer Velocity (rps)", () -> this.setpoint.serializerVelocityRotationsPerSecond());

    ShuffleboardLayout goal = Telemetry.addColumn(tab, "Goal");

    goal.addDouble(
        "Shoulder Position (deg)",
        () -> Units.rotationsToDegrees(this.goal.shoulderAngleRotations().position));
    goal.addDouble(
        "Shoulder Velocity (dps)",
        () -> Units.rotationsToDegrees(this.goal.shoulderAngleRotations().velocity));

    goal.addDouble("Roller Velocity (rps)", () -> this.goal.rollerVelocityRotationsPerSecond());

    goal.addDouble("Flywheel Velocity (rps)", () -> this.goal.flywheelVelocityRotationsPerSecond());

    goal.addDouble(
        "Serializer Velocity (rps)", () -> this.goal.serializerVelocityRotationsPerSecond());
  }

  private void updateMeasurement() {
    State measuredShoulderState = arm.getMeasuredShoulderState();

    double measuredIntakeRollerVelocity = intake.getRollerVelocity();

    double measuredShooterFlywheelVelocity = shooter.getFlywheelVelocity();
    double measuredShooterSerializerVelocity = shooter.getSerializerVelocity();

    measurement =
        new SuperstructureState(
            measuredShoulderState,
            measuredIntakeRollerVelocity,
            measuredShooterFlywheelVelocity,
            measuredShooterSerializerVelocity);

    SuperstructureMechanism.getInstance().updateSuperstructure(measurement);
  }

  public SuperstructureState getState() {
    updateMeasurement();

    return measurement;
  }

  private void updateSetpoint() {
    setpoint = SuperstructureState.nextSetpoint(setpoint, goal);

    arm.setSetpoint(
        setpoint.shoulderAngleRotations().position, setpoint.shoulderAngleRotations().velocity);

    shooter.setSetpoint(
        setpoint.flywheelVelocityRotationsPerSecond(),
        setpoint.serializerVelocityRotationsPerSecond());

    intake.setSetpoint(setpoint.rollerVelocityRotationsPerSecond());
  }

  public void setPosition(SuperstructureState state) {
    arm.setShoulderPosition(state.shoulderAngleRotations().position);
  }

  public void setGoal(SuperstructureState goal) {
    this.goal = goal;
  }

  public boolean at(SuperstructureState goal) {
    updateMeasurement();

    return measurement.at(goal);
  }

  private Command to(SuperstructureState goal) {
    return new ToGoal(goal);
  }

  public Command stow() {
    return to(SuperstructureState.STOW);
  }

  public Command intake() {
    return to(SuperstructureState.INTAKE)
        .andThen(
            to(
                SuperstructureState.INTAKE
                    .withSerializerVelocity(SerializerConstants.INTAKE_VELOCITY)
                    .withRollerVelocity(RollerConstants.INTAKE_VELOCITY)));
  }

  public Command shoot() {
    return to(SuperstructureState.SHOOT)
        .andThen(
            to(SuperstructureState.SHOOT.withFlywheelVelocity(FlywheelConstants.SPEAKER_VELOCITY))
                .withTimeout(1.0))
        .andThen(
            to(
                SuperstructureState.SHOOT
                    .withFlywheelVelocity(FlywheelConstants.SPEAKER_VELOCITY)
                    .withSerializerVelocity(SerializerConstants.SPEAKER_SERIALIZER_VELOCITY)));
  }

  public Command amp() {
    return to(SuperstructureState.AMP)
        .andThen(to(SuperstructureState.AMP_SHOOT).withTimeout(1.0))
        .andThen(to(SuperstructureState.AMP_SERIALIZE));
  }
}
