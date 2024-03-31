package frc.robot.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.RobotConstants;

/** Simulated flywheel motor. */
public class FlywheelMotorIOSim implements FlywheelMotorIO {

  private final FlywheelSim flywheelSim;

  private final SimpleMotorFeedforward flywheelVelocityFeedforward;

  public FlywheelMotorIOSim() {
    flywheelSim = new FlywheelSim(DCMotor.getKrakenX60(1), 2.25, 0.009272);

    flywheelVelocityFeedforward = new SimpleMotorFeedforward(0.0, 0.2685);
  }

  @Override
  public void configure() {}

  @Override
  public void update(FlywheelMotorIOValues values) {
    values.velocityRotationsPerSecond = getFlywheelVelocityRotationsPerSecond();
    values.currentAmps = flywheelSim.getCurrentDrawAmps();
  }

  @Override
  public void setSetpoint(double velocityRotationsPerSecond) {
    double volts = flywheelVelocityFeedforward.calculate(velocityRotationsPerSecond);

    flywheelSim.setInputVoltage(volts);

    flywheelSim.update(RobotConstants.PERIODIC_DURATION);
  }

  private double getFlywheelVelocityRotationsPerSecond() {
    return flywheelSim.getAngularVelocityRPM() / 60;
  }
}
