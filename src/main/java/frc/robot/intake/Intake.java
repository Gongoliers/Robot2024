package frc.robot.intake;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.robot.intake.IntakeConstants.IntakeCommandConstants;
import frc.robot.intake.IntakeConstants.RollerMotorConstants;
import frc.robot.intake.RollerMotorIO.RollerMotorIOValues;

/** Subsystem class for the intake subsystem. */
public class Intake extends Subsystem {

  /** Instance variable for the intake subsystem singleton. */
  private static Intake instance = null;

  /** Roller motor. */
  private final RollerMotorIO rollerMotor;

  /** Roller motor values. */
  private final RollerMotorIOValues rollerMotorValues = new RollerMotorIOValues();

  private final MedianFilter rollerMotorCurrentFilter = new MedianFilter(5);

  /** Creates a new instance of the intake subsystem. */
  private Intake() {
    rollerMotor = IntakeFactory.createRollerMotor();
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
    rollerMotor.update(rollerMotorValues);

    rollerMotorCurrentFilter.calculate(rollerMotorValues.currentAmps);
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    ShuffleboardLayout roller = Telemetry.addColumn(tab, "Roller");

    roller.addDouble("Current (A)", () -> rollerMotorValues.currentAmps);
    roller.addDouble(
        "Rotor Velocity (rps)", () -> rollerMotorValues.angularVelocityRotationsPerSecond);
    roller.addDouble("Roller Velocity (rps)", this::getRollerVelocity);
    roller.addBoolean("Current Spike?", this::rollerCurrentSpike);
    roller.addBoolean("Slowed Down?", this::rollerSlowedDown);
    roller.addBoolean("Has Note?", this::hasNote);
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
   * Gets the velocity of the rollers in rotations per second.
   *
   * @return the velocity of the rollers in rotations per second.
   */
  private double getRollerVelocity() {
    return rollerMotorValues.angularVelocityRotationsPerSecond / RollerMotorConstants.GEARING;
  }

  /**
   * Returns true if the roller speed drops below the speed for intaking notes.
   *
   * @return true if the roller speed drops below the speed for intaking notes.
   */
  private boolean rollerSlowedDown() {
    return Math.abs(getRollerVelocity()) < RollerMotorConstants.NOTE_SPEED;
  }

  /**
   * Returns true if the rollers have a note.
   *
   * @return true if the rollers have a note.
   */
  private boolean hasNote() {
    return rollerCurrentSpike() || rollerSlowedDown();
  }

  public Command smartIntake() {
    return run(() -> rollerMotor.setVoltage(RollerMotorConstants.INTAKE_VOLTAGE))
        .raceWith(
            Commands.waitSeconds(IntakeCommandConstants.NOTE_DETECTION_DELAY)
                .andThen(Commands.waitUntil(this::hasNote)))
        .finallyDo(rollerMotor::stop);
  }

  public Command outtake() {
    return Commands.run(() -> rollerMotor.setVoltage(-RollerMotorConstants.INTAKE_VOLTAGE))
        .finallyDo(rollerMotor::stop);
  }
}
