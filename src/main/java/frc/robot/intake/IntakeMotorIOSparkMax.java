package frc.robot.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.lib.Configurator;
import frc.robot.intake.IntakeConstants.IntakeMotorConstants;

/** Intake motor using a Spark Max. */
public class IntakeMotorIOSparkMax implements IntakeMotorIO {

  /** Hardware Spark Max. */
  private final CANSparkMax sparkMax;

  public IntakeMotorIOSparkMax() {
    sparkMax = new CANSparkMax(IntakeMotorConstants.ID.id(), MotorType.kBrushless);
  }

  @Override
  public void configure() {
    Configurator.configureREV(sparkMax::restoreFactoryDefaults);

    sparkMax.setInverted(IntakeMotorConstants.IS_INVERTED);

    Configurator.configureREV(
        () ->
            sparkMax.setSmartCurrentLimit((int) IntakeMotorConstants.CURRENT_LIMITS.breakerAmps()));
  }

  @Override
  public void update(IntakeMotorIOValues values) {
    values.angularVelocityRotationsPerSecond = sparkMax.getEncoder().getVelocity() / 60.0;
    values.currentAmps = sparkMax.getOutputCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    sparkMax.setVoltage(volts);
  }

  @Override
  public void stop() {
    setVoltage(0);
  }
}
