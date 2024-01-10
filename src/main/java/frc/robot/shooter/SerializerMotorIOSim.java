package frc.robot.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.RobotConstants;
import frc.robot.shooter.ShooterConstants.SerializerConstants;

/** Simulated serializer motor. */
public class SerializerMotorIOSim implements SerializerMotorIO {

  /** Represents the motor used the drive the serializer. */
  private final DCMotor motorSim = DCMotor.getNeo550(1);

  /** Represents the serializer driven by the motor. */
  private final FlywheelSim serializerSim =
      new FlywheelSim(motorSim, SerializerConstants.GEARING, SerializerConstants.MOI);

  @Override
  public void configure() {}

  @Override
  public void update(SerializerMotorIOValues values) {
    serializerSim.update(RobotConstants.TICK_PERIOD);

    values.angularVelocityRotationsPerSecond = serializerSim.getAngularVelocityRPM() / 60.0;
    values.currentAmps = serializerSim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    serializerSim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    serializerSim.setInputVoltage(0);
  }
}
