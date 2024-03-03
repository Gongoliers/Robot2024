package frc.robot.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.lib.Configurator;

/** Serializer motor using Spark Max. */
public class SerializerMotorIOSparkMax implements SerializerMotorIO {

  /** Hardware Spark Max for leading. */
  private final CANSparkMax sparkMax;

  public SerializerMotorIOSparkMax() {
    sparkMax = new CANSparkMax(4, MotorType.kBrushless);
  }

  @Override
  public void configure() {
    Configurator.configureREV(sparkMax::restoreFactoryDefaults);

    Configurator.configureREV(() -> sparkMax.setSmartCurrentLimit(40));

    Configurator.configureStatusFrames(sparkMax);
  }

  @Override
  public void update(SerializerMotorIOValues values) {
    values.angularVelocityRotationsPerSecond = sparkMax.getEncoder().getVelocity();
    values.currentAmps = sparkMax.getOutputCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    sparkMax.setVoltage(volts);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
