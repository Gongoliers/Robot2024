package frc.lib.sensor;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;
import frc.lib.config.Configurator;

/** Pigeon 2 gyroscope. */
public class GyroscopeIOPigeon2 implements GyroscopeIO {

  /** Pigeon 2. */
  private final Pigeon2 gyroscope;

  /** Pigeon 2 status signals. */
  private final StatusSignal<Double> roll, pitch, yaw, rollVelocity, pitchVelocity, yawVelocity;

  public GyroscopeIOPigeon2() {
    gyroscope = new Pigeon2(0, "swerve");

    roll = gyroscope.getRoll();
    pitch = gyroscope.getPitch();
    yaw = gyroscope.getYaw();

    rollVelocity = gyroscope.getAngularVelocityXWorld();
    pitchVelocity = gyroscope.getAngularVelocityYWorld();
    yawVelocity = gyroscope.getAngularVelocityZWorld();
  }

  @Override
  public void configure() {
    Configurator.configurePigeon2(gyroscope.getConfigurator(), new Pigeon2Configuration());
  }

  @Override
  public void update(GyroscopeIOValues values) {
    BaseStatusSignal.refreshAll(roll, pitch, yaw, rollVelocity, pitchVelocity, yawVelocity);

    values.rollRotations = Units.degreesToRotations(roll.getValue());
    values.pitchRotations = Units.degreesToRotations(pitch.getValue());
    values.yawRotations = Units.degreesToRotations(yaw.getValue());

    values.rollVelocityRotations = Units.degreesToRotations(rollVelocity.getValue());
    values.pitchVelocityRotations = Units.degreesToRotations(pitchVelocity.getValue());
    values.yawVelocityRotations = Units.degreesToRotations(yawVelocity.getValue());
  }

  @Override
  public void setYaw(double yawRotations) {
    gyroscope.setYaw(Units.rotationsToDegrees(yawRotations));
  }
}
