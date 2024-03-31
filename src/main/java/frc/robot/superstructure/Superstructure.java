package frc.robot.superstructure;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.robot.RobotMechanisms;
import frc.robot.arm.Arm;
import frc.robot.intake.Intake;
import frc.robot.shooter.Shooter;

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
        "Wrist Position (deg)",
        () -> Units.rotationsToDegrees(this.measurement.wristAngleRotations().position));
    measurement.addDouble(
        "Wrist Velocity (dps)",
        () -> Units.rotationsToDegrees(this.measurement.wristAngleRotations().velocity));

    measurement.addDouble(
        "Pivot Position (deg)",
        () -> Units.rotationsToDegrees(this.measurement.pivotAngleRotations().position));
    measurement.addDouble(
        "Pivot Velocity (dps)",
        () -> Units.rotationsToDegrees(this.measurement.pivotAngleRotations().velocity));

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
        "Wrist Position (deg)",
        () -> Units.rotationsToDegrees(this.setpoint.wristAngleRotations().position));
    setpoint.addDouble(
        "Wrist Velocity (dps)",
        () -> Units.rotationsToDegrees(this.setpoint.wristAngleRotations().velocity));

    setpoint.addDouble(
        "Pivot Position (deg)",
        () -> Units.rotationsToDegrees(this.setpoint.pivotAngleRotations().position));
    setpoint.addDouble(
        "Pivot Velocity (dps)",
        () -> Units.rotationsToDegrees(this.setpoint.pivotAngleRotations().velocity));

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

    goal.addDouble(
        "Wrist Position (deg)",
        () -> Units.rotationsToDegrees(this.goal.wristAngleRotations().position));
    goal.addDouble(
        "Wrist Velocity (dps)",
        () -> Units.rotationsToDegrees(this.goal.wristAngleRotations().velocity));

    goal.addDouble(
        "Pivot Position (deg)",
        () -> Units.rotationsToDegrees(this.goal.pivotAngleRotations().position));
    goal.addDouble(
        "Pivot Velocity (dps)",
        () -> Units.rotationsToDegrees(this.goal.pivotAngleRotations().velocity));

    goal.addDouble("Roller Velocity (rps)", () -> this.goal.rollerVelocityRotationsPerSecond());

    goal.addDouble("Flywheel Velocity (rps)", () -> this.goal.flywheelVelocityRotationsPerSecond());

    goal.addDouble(
        "Serializer Velocity (rps)", () -> this.goal.serializerVelocityRotationsPerSecond());
  }

  private void updateMeasurement() {
    State measuredShoulderState = arm.getMeasuredShoulderState();
    State measuredWristState = arm.getMeasuredWristState();

    State measuredIntakePivotState = intake.getMeasuredPivotState();
    double measuredIntakeRollerVelocity = intake.getRollerVelocity();

    double measuredShooterFlywheelVelocity = shooter.getFlywheelVelocity();
    double measuredShooterSerializerVelocity = shooter.getSerializerVelocity();

    measurement =
        new SuperstructureState(
            measuredShoulderState,
            measuredWristState,
            measuredIntakePivotState,
            measuredIntakeRollerVelocity,
            measuredShooterFlywheelVelocity,
            measuredShooterSerializerVelocity);

    RobotMechanisms.getInstance().updateSuperstructure(measurement);
  }

  public SuperstructureState getState() {
    updateMeasurement();

    return measurement;
  }

  private void updateSetpoint() {
    setpoint = SuperstructureState.nextSetpoint(setpoint, goal);

    arm.setShoulderSetpoint(
        setpoint.shoulderAngleRotations().position, setpoint.shoulderAngleRotations().velocity);
    arm.setWristSetpoint(
        setpoint.wristAngleRotations().position, setpoint.wristAngleRotations().velocity);

    intake.setPivotSetpoint(
        setpoint.pivotAngleRotations().position, setpoint.pivotAngleRotations().velocity);
  }

  public void setPosition(SuperstructureState state) {
    arm.setShoulderPosition(state.shoulderAngleRotations().position);
    arm.setWristPosition(state.wristAngleRotations().position);
    intake.setPivotPosition(state.pivotAngleRotations().position);
  }

  public void setGoal(SuperstructureState goal) {
    this.goal = goal;

    // resetMotionProfile();
  }

  // private void resetMotionProfile() {
  //   setpoint = measurement;
  // }

  public boolean at(SuperstructureState goal) {
    updateMeasurement();

    return measurement.at(goal);
  }

  public Command to(SuperstructureState goal) {
    return new ToGoal(goal);
  }

  public Command initial() {
    return to(SuperstructureState.INITIAL);
  }

  public Command stow() {
    return to(SuperstructureState.STOW);
  }

  public Command intake() {
    return to(SuperstructureState.INTAKE);
  }

  public Command shoot() {
    return to(SuperstructureState.SHOOT);
  }

  public Command amp() {
    return to(SuperstructureState.AMP);
  }
}
