package frc.robot.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.lib.Configurator;

/** Roller motor using a Spark Max. */
public class RollerMotorIOSparkMax implements RollerMotorIO {

  /** Hardware Spark Max. */
  private final CANSparkMax sparkMax;

  public RollerMotorIOSparkMax() {
    sparkMax = new CANSparkMax(5, MotorType.kBrushless);
  }

  @Override
  public void configure() {
    Configurator.configureREV(sparkMax::restoreFactoryDefaults);

    sparkMax.setInverted(false);

    Configurator.configureREV(() -> sparkMax.setSmartCurrentLimit(40));

    Configurator.configureStatusFrames(sparkMax);
  }

  @Override
  public void update(RollerMotorIOValues values) {
    values.velocityRotationsPerSecond = getVelocity();
    values.currentAmps = sparkMax.getOutputCurrent();
  }

  @Override
  public void setSetpoint(double velocityRotationsPerSecond) {
    // TODO Implement velocity setpoint following
  }

  public double getVelocity() {
    return sparkMax.getEncoder().getVelocity() / 60.0;
  }
}
