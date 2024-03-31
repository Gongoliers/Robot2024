package frc.robot.intake;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.RobotConstants;

/** Simulated roller motor. */
public class RollerMotorIOSim implements RollerMotorIO {

  private final FlywheelSim rollerSim;

  private final SimpleMotorFeedforward rollerVelocityFeedforward;

  public RollerMotorIOSim() {
    rollerSim = new FlywheelSim(DCMotor.getKrakenX60(1), 3.0, 0.03472);

    rollerVelocityFeedforward = new SimpleMotorFeedforward(0.0, 0.358);
  }

  @Override
  public void configure() {}

  @Override
  public void update(RollerMotorIOValues values) {
    values.velocityRotationsPerSecond = getRollerVelocityRotationsPerSecond();
    values.currentAmps = rollerSim.getCurrentDrawAmps();
  }

  @Override
  public void setSetpoint(double velocityRotationsPerSecond) {
    double volts = rollerVelocityFeedforward.calculate(velocityRotationsPerSecond);

    rollerSim.setInputVoltage(volts);

    rollerSim.update(RobotConstants.PERIODIC_DURATION);
  }

  private double getRollerVelocityRotationsPerSecond() {
    return rollerSim.getAngularVelocityRPM() / 60;
  }
}
