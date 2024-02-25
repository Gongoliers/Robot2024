package frc.robot.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.robot.RobotConstants;
import frc.robot.RobotMechanisms;
import frc.robot.intake.IntakeConstants.PivotMotorConstants;
import frc.robot.intake.IntakeConstants.RollerMotorConstants;
import frc.robot.intake.PivotMotorIO.PivotMotorIOValues;
import frc.robot.intake.RollerMotorIO.RollerMotorIOValues;
import java.util.function.DoubleSupplier;

/** Subsystem class for the intake subsystem. */
public class Intake extends Subsystem {

  /** Instance variable for the intake subsystem singleton. */
  private static Intake instance = null;

  /** Pivot motor. */
  private final PivotMotorIO pivotMotor;

  /** Pivot motor values. */
  private final PivotMotorIOValues pivotMotorValues = new PivotMotorIOValues();

  /** Pivot motor goal. */
  private TrapezoidProfile.State pivotGoal;

  /** Pivot motor setpoint. */
  private TrapezoidProfile.State pivotSetpoint;

  /** Roller motor. */
  private final RollerMotorIO rollerMotor;

  /** Roller motor values. */
  private final RollerMotorIOValues rollerMotorValues = new RollerMotorIOValues();

  /** Roller motor current median filter. */
  private final MedianFilter rollerMotorCurrentFilter = new MedianFilter(3);

  /** Roller motor current spike debouncer. */
  private final Debouncer rollerMotorCurrentSpikeDebouncer =
      new Debouncer(RollerMotorConstants.STALL_DURATION);

  /** Creates a new instance of the intake subsystem. */
  private Intake() {
    pivotMotor = IntakeFactory.createPivotMotor();
    rollerMotor = IntakeFactory.createRollerMotor();

    pivotMotor.configure();
    rollerMotor.configure();

    Rotation2d initialAngle = PivotMotorConstants.MAXIMUM_ANGLE;

    pivotMotor.setPosition(initialAngle.getRotations());
    pivotMotor.update(pivotMotorValues);

    pivotGoal = new TrapezoidProfile.State(initialAngle.getRotations(), 0);
    pivotSetpoint = new TrapezoidProfile.State(initialAngle.getRotations(), 0);
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

    updatePivotSetpoint();

    rollerMotor.update(rollerMotorValues);

    rollerMotorCurrentFilter.calculate(rollerMotorValues.currentAmps);

    RobotMechanisms.getInstance()
        .setIntakeAngle(Rotation2d.fromRotations(pivotMotorValues.positionRotations));
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    ShuffleboardLayout pivot = Telemetry.addColumn(tab, "Pivot");

    pivot.addDouble(
        "Position (deg)", () -> Units.rotationsToDegrees(pivotMotorValues.positionRotations));
    pivot.addDouble("Setpoint (deg)", () -> Units.rotationsToDegrees(pivotSetpoint.position));
    pivot.addDouble("Goal (deg)", () -> Units.rotationsToDegrees(pivotGoal.position));
    pivot.addBoolean("Is Not Stowed?", this::isNotStowed);

    ShuffleboardLayout roller = Telemetry.addColumn(tab, "Roller");

    roller.addDouble("Current (A)", () -> rollerMotorValues.currentAmps);
    roller.addDouble(
        "Motor Velocity (rps)", () -> rollerMotorValues.angularVelocityRotationsPerSecond);
    roller.addDouble("Roller Velocity (rps)", this::getRollerVelocity);
    roller.addBoolean("Current Spike?", this::rollerCurrentSpike);
    roller.addBoolean("Stalled?", this::rollerStalled);
  }

  public Command drivePivot(DoubleSupplier voltageSupplier) {
    return run(() -> pivotMotor.setVoltage(voltageSupplier.getAsDouble()))
        .finallyDo(pivotMotor::stop);
  }

  public void setPivotGoal(Rotation2d goal) {
    pivotGoal = new TrapezoidProfile.State(goal.getRotations(), 0);
  }

  private void updatePivotSetpoint() {
    pivotSetpoint =
        PivotMotorConstants.MOTION_PROFILE.calculate(
            RobotConstants.PERIODIC_DURATION, pivotSetpoint, pivotGoal);

    pivotMotor.setSetpoint(pivotSetpoint.position, pivotSetpoint.velocity);
  }

  private boolean atPivotGoal() {
    return atPivotSetpoint() && pivotGoal.equals(pivotSetpoint);
  }

  private boolean atPivotSetpoint() {
    pivotMotor.update(pivotMotorValues);

    return MathUtil.isNear(
        pivotSetpoint.position,
        pivotMotorValues.positionRotations,
        PivotMotorConstants.TOLERANCE.getRotations());
  }

  public Command out() {
    return pivotTo(PivotMotorConstants.MINIMUM_ANGLE);
  }

  public Command in() {
    return pivotTo(PivotMotorConstants.MAXIMUM_ANGLE);
  }

  private Command pivotTo(Rotation2d angle) {
    return runOnce(() -> setPivotGoal(angle)).andThen(Commands.waitUntil(this::atPivotGoal));
  }

  public boolean isNotStowed() {
    return pivotMotorValues.positionRotations < PivotMotorConstants.OUT_ANGLE.getRotations();
  }

  public Command intake() {
    return Commands.run(() -> rollerMotor.setVoltage(RollerMotorConstants.INTAKE_VOLTAGE))
        .finallyDo(rollerMotor::stop);
  }

  /**
   * Returns true if the roller motor has a current spike.
   *
   * @return true if the roller motor has a current spike.
   */
  private boolean rollerCurrentSpike() {
    return rollerMotorCurrentFilter.calculate(rollerMotorValues.currentAmps)
        > RollerMotorConstants.NOTE_CURRENT;
  }

  /**
   * Returns true if the roller motor is stalled.
   *
   * @return true if the roller motor is stalled.
   */
  private boolean rollerStalled() {
    return rollerMotorCurrentSpikeDebouncer.calculate(rollerCurrentSpike());
  }

  /**
   * Gets the velocity of the rollers in rotations per second.
   *
   * @return the velocity of the rollers in rotations per second.
   */
  private double getRollerVelocity() {
    return rollerMotorValues.angularVelocityRotationsPerSecond / RollerMotorConstants.GEARING;
  }

  public Command smartIntake() {
    return Commands.repeatingSequence(
            run(() -> rollerMotor.setVoltage(RollerMotorConstants.INTAKE_VOLTAGE))
                .until(this::rollerStalled),
            run(() -> rollerMotor.setVoltage(RollerMotorConstants.OUTTAKE_VOLTAGE))
                .until(() -> !rollerStalled()))
        .finallyDo(rollerMotor::stop);
  }

  public Command outtake() {
    return Commands.run(() -> rollerMotor.setVoltage(RollerMotorConstants.OUTTAKE_VOLTAGE))
        .finallyDo(rollerMotor::stop);
  }
}
