package frc.robot.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.lib.TrapezoidProfileTelemetry;
import frc.robot.RobotConstants;
import frc.robot.RobotMechanisms;
import frc.robot.intake.IntakeConstants.PivotMotorConstants;
import frc.robot.intake.IntakeConstants.RollerMotorConstants;
import frc.robot.intake.PivotMotorIO.PivotMotorIOValues;
import frc.robot.intake.RollerMotorIO.RollerMotorIOValues;

/** Subsystem class for the intake subsystem. */
public class Intake extends Subsystem {

  /** Instance variable for the intake subsystem singleton. */
  private static Intake instance = null;

  private final RobotMechanisms mechanism;

  /** Pivot motor. */
  private final PivotMotorIO pivotMotor;

  /** Pivot motor values. */
  private final PivotMotorIOValues pivotMotorValues = new PivotMotorIOValues();

  /** Pivot motor goal. */
  private TrapezoidProfile.State pivotGoal;

  /** Pivot motor setpoint. */
  private TrapezoidProfile.State pivotSetpoint;

  /** Telemetry for the pivot trapezoid profile. */
  private final TrapezoidProfileTelemetry pivotProfileTelemetry;

  /** Roller motor. */
  private final RollerMotorIO rollerMotor;

  /** Roller motor values. */
  private final RollerMotorIOValues rollerMotorValues = new RollerMotorIOValues();

  /** Creates a new instance of the intake subsystem. */
  private Intake() {
    mechanism = RobotMechanisms.getInstance();

    pivotMotor = IntakeFactory.createPivotMotor();
    rollerMotor = IntakeFactory.createRollerMotor();

    pivotMotor.configure();
    rollerMotor.configure();

    Rotation2d initialAngle = PivotMotorConstants.MAXIMUM_ANGLE;

    pivotMotor.setPosition(initialAngle.getRotations());
    pivotMotor.update(pivotMotorValues);

    pivotGoal = new TrapezoidProfile.State(initialAngle.getRotations(), 0);
    pivotSetpoint = new TrapezoidProfile.State(initialAngle.getRotations(), 0);

    pivotProfileTelemetry = new TrapezoidProfileTelemetry("intake/pivotProfile");
  }

  /**
   * Gets the instance of the intake subsystem.
   *
   * @return the instance of the intake subsystem.
   */
  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }

    return instance;
  }

  @Override
  public void periodic() {
    pivotMotor.update(pivotMotorValues);

    rollerMotor.update(rollerMotorValues);

    mechanism.updateIntake(
        Rotation2d.fromRotations(pivotMotorValues.positionRotations),
        rollerMotorValues.velocityRotationsPerSecond);

    pivotProfileTelemetry.update(getPivotMeasuredState(), getPivotSetpoint(), getPivotGoal());
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    ShuffleboardLayout pivot = Telemetry.addColumn(tab, "Pivot");

    pivot.addDouble(
        "Position (deg)", () -> Units.rotationsToDegrees(getPivotMeasuredState().position));
    pivot.addDouble("Setpoint (deg)", () -> Units.rotationsToDegrees(getPivotSetpoint().position));
    pivot.addDouble("Goal (deg)", () -> Units.rotationsToDegrees(getPivotGoal().position));
    pivot.addBoolean("Is Not Stowed?", this::isOut);

    ShuffleboardLayout roller = Telemetry.addColumn(tab, "Roller");

    roller.addDouble("Current (A)", () -> rollerMotorValues.currentAmps);
    roller.addDouble("Velocity (rps)", () -> rollerMotorValues.velocityRotationsPerSecond);
  }

  /**
   * Returns the intake pivot's measured state.
   *
   * @return the intake pivot's measured state.
   */
  public State getPivotMeasuredState() {
    pivotMotor.update(pivotMotorValues);

    return new State(
        pivotMotorValues.positionRotations, pivotMotorValues.velocityRotationsPerSecond);
  }

  /**
   * Sets the intake pivot's goal.
   *
   * @param goal the intake pivot's goal.
   */
  public void setPivotGoal(Rotation2d goal) {
    setPivotGoal(new TrapezoidProfile.State(goal.getRotations(), 0.0));
  }

  /**
   * Sets the intake pivot's goal.
   *
   * @param goal the intake pivot's goal.
   */
  public void setPivotGoal(State goal) {
    pivotGoal = goal;
  }

  /**
   * Returns the intake pivot's goal.
   *
   * @return the intake pivot's goal.
   */
  public State getPivotGoal() {
    return pivotGoal;
  }

  /**
   * Sets the intake pivot's setpoint.
   *
   * @param setpoint the intake pivot's setpoint.
   */
  public void setPivotSetpoint(State setpoint) {
    pivotSetpoint = setpoint;
  }

  /**
   * Returns the intake pivot's setpoint.
   *
   * @return the intake pivot's setpoint.
   */
  public State getPivotSetpoint() {
    return pivotSetpoint;
  }

  /**
   * Applies a pivot setpoint to the pivot motor's controller.
   *
   * @param setpoint the pivot's setpoint.
   */
  public void applyPivotSetpoint(State setpoint) {
    pivotMotor.setSetpoint(setpoint.position, setpoint.velocity);
  }

  /**
   * Returns true if the intake pivot is at its goal.
   *
   * @return true if the intake pivot is at its goal.
   */
  private boolean atPivotGoal() {
    return atPivotSetpoint() && pivotGoal.equals(pivotSetpoint);
  }

  /**
   * Returns true if the intake pivot is at its setpoint.
   *
   * @return true if the intake pivot is at its setpoint.
   */
  private boolean atPivotSetpoint() {
    pivotMotor.update(pivotMotorValues);

    return MathUtil.isNear(
        pivotSetpoint.position,
        pivotMotorValues.positionRotations,
        PivotMotorConstants.TOLERANCE.getRotations());
  }

  /**
   * Returns a command that pivots the intake to an angle.
   *
   * @param angle the angle to pivot the intake to.
   * @return a command that pivots the intake to an angle.
   */
  private Command pivotTo(Rotation2d angle) {
    return run(() -> {
          State nextPivotSetpoint =
              PivotMotorConstants.MOTION_PROFILE.calculate(
                  RobotConstants.PERIODIC_DURATION, pivotSetpoint, pivotGoal);

          setPivotSetpoint(nextPivotSetpoint);
          applyPivotSetpoint(nextPivotSetpoint);
        })
        .beforeStarting(() -> setPivotGoal(angle))
        .until(this::atPivotGoal);
  }

  /**
   * Returns a command that pivots the intake to the unstowed position.
   *
   * @return a command that pivots the intake to the unstowed position.
   */
  public Command unstow() {
    return pivotTo(PivotMotorConstants.MINIMUM_ANGLE);
  }

  /**
   * Returns a command that pivots the intake to the stowed position.
   *
   * @return a command that pivots the intake to the stowed position.
   */
  public Command stow() {
    return pivotTo(PivotMotorConstants.MAXIMUM_ANGLE);
  }

  /**
   * Returns true if the intake is out (not stowed).
   *
   * @return true if the intake is out (not stowed).
   */
  public boolean isOut() {
    return pivotMotorValues.positionRotations < PivotMotorConstants.OUT_ANGLE.getRotations();
  }

  /**
   * Returns a command that intakes using the rollers.
   *
   * @return a command that intakes using the rollers.
   */
  public Command intake() {
    return Commands.runEnd(
        () -> rollerMotor.setSetpoint(RollerMotorConstants.INTAKE_VELOCITY),
        () -> rollerMotor.setSetpoint(0.0));
  }

  /**
   * Returns a command that outtakes using the rollers.
   *
   * @return a command that outtakes using the rollers.
   */
  public Command outtake() {
    return Commands.runEnd(
        () -> rollerMotor.setSetpoint(RollerMotorConstants.OUTTAKE_VELOCITY),
        () -> rollerMotor.setSetpoint(0.0));
  }
}
