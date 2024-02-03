package frc.robot.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.lib.Configurator;

/** Flywheel motor using Spark Maxes. */
public class FlywheelMotorIOSparkMax implements FlywheelMotorIO {

  /** Hardware Spark Max for leading. */
  private final CANSparkMax leaderSparkMax;

  /** Hardware Spark Max for following. */
  private final CANSparkMax followerSparkMax;

  /** Creates a new flywheel motor using Spark Maxes. */
  public FlywheelMotorIOSparkMax() {
    leaderSparkMax = new CANSparkMax(19, MotorType.kBrushless);
    followerSparkMax = new CANSparkMax(20, MotorType.kBrushless);
  }

  @Override
  public void configure() {
    Configurator.configureREV(leaderSparkMax::restoreFactoryDefaults);
    Configurator.configureREV(followerSparkMax::restoreFactoryDefaults);

    followerSparkMax.follow(leaderSparkMax, true);
  }

  @Override
  public void update(FlywheelMotorIOValues values) {
    values.angularVelocityRotationsPerSecond = leaderSparkMax.getEncoder().getVelocity() / 60.0;
    values.currentAmps = leaderSparkMax.getOutputCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    leaderSparkMax.setVoltage(volts);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
