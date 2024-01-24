package frc.robot.intake;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Subsystem;
import frc.robot.intake.IntakeConstants.IntakeMotorConstants;
import frc.robot.intake.IntakeMotorIO.IntakeMotorIOValues;

/** Subsystem class for the intake subsystem. */
public class Intake extends Subsystem {

  /** Instance variable for the intake subsystem singleton. */
  private static Intake instance = null;

  /** Intake motor. */
  private final IntakeMotorIO intakeMotor;

  /** Intake motor values. */
  private final IntakeMotorIOValues intakeMotorValues = new IntakeMotorIOValues();

  /** Creates a new instance of the intake subsystem. */
  private Intake() {
    intakeMotor = IntakeFactory.createIntakeMotor();
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
    intakeMotor.update(intakeMotorValues);
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    tab.addDouble("Current", () -> intakeMotorValues.currentAmps);
  }

  public Command intake() {
    return Commands.run(() -> intakeMotor.setVoltage(IntakeMotorConstants.INTAKE_VOLTAGE))
        .finallyDo(intakeMotor::stop);
  }

  public Command outtake() {
    return Commands.run(() -> intakeMotor.setVoltage(-IntakeMotorConstants.INTAKE_VOLTAGE))
        .finallyDo(intakeMotor::stop);
  }
}
