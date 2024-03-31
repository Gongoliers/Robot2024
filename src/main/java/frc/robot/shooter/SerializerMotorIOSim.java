package frc.robot.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.RobotConstants;

/** Simulated serializer motor. */
public class SerializerMotorIOSim implements SerializerMotorIO {

  private final FlywheelSim serializerSim;

  private final SimpleMotorFeedforward serializerVelocityFeedforward;

  public SerializerMotorIOSim() {
    serializerSim = new FlywheelSim(DCMotor.getKrakenX60(1), 3.0, 0.06202);

    serializerVelocityFeedforward = new SimpleMotorFeedforward(0.0, 0.358);
  }

  @Override
  public void configure() {}

  @Override
  public void update(SerializerMotorIOValues values) {
    values.velocityRotationsPerSecond = getSerializerVelocityRotationsPerSecond();
    values.currentAmps = serializerSim.getCurrentDrawAmps();
  }

  @Override
  public void setSetpoint(double velocityRotationsPerSecond) {
    double volts = serializerVelocityFeedforward.calculate(velocityRotationsPerSecond);

    serializerSim.setInputVoltage(volts);

    serializerSim.update(RobotConstants.PERIODIC_DURATION);
  }

  private double getSerializerVelocityRotationsPerSecond() {
    return serializerSim.getAngularVelocityRPM() / 60;
  }
}
