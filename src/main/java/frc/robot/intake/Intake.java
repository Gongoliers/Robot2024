package frc.robot.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
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

  /** Roller motor current median filter. */
  private final MedianFilter rollerMotorCurrentFilter = new MedianFilter(3);

  /** Roller motor current spike debouncer. */
  private final Debouncer rollerMotorCurrentSpikeDebouncer =
      new Debouncer(RollerMotorConstants.STALL_DURATION);

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
        "Motor Velocity (rps)", () -> rollerMotorValues.angularVelocityRotationsPerSecond);
    roller.addDouble("Roller Velocity (rps)", this::getRollerVelocity);
    roller.addBoolean("Current Spike?", this::rollerCurrentSpike);
    roller.addBoolean("Stalled?", this::rollerStalled);
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
