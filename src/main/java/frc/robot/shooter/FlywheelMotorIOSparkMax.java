package frc.robot.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.lib.Configurator;

/** Flywheel motor using Spark Maxes. */
public class FlywheelMotorIOSparkMax implements FlywheelMotorIO {

  /** Hardware Spark Max for leading. */
  private final CANSparkMax leader;

  /** Hardware Spark Max for following. */
  private final CANSparkMax follower;

  /** Creates a new flywheel motor using Spark Maxes. */
  public FlywheelMotorIOSparkMax() {
    leader = new CANSparkMax(19, MotorType.kBrushless);
    follower = new CANSparkMax(20, MotorType.kBrushless);
  }

  @Override
  public void configure() {
    Configurator.configureREV(leader::restoreFactoryDefaults);
    Configurator.configureREV(follower::restoreFactoryDefaults);

    follower.follow(leader, true);
  }

  @Override
  public void update(FlywheelMotorIOValues values) {
    values.angularVelocityRotationsPerSecond = leader.getEncoder().getVelocity() / 60.0;
    values.currentAmps = follower.getOutputCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
