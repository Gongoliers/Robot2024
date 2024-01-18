package frc.robot.odometry;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;
import frc.lib.CAN;
import frc.lib.ConfigApplier;

/** Pigeon 2 gyroscope. */
public class GyroscopeIOPigeon2 implements GyroscopeIO {

  /** Hardware Pigeon 2. */
  private final Pigeon2 pigeon2;

  /** Pigeon 2's roll, pitch, and yaw status signals. */
  private final StatusSignal<Double> roll, pitch, yaw;

  /**
   * Creates a new Pigeon 2 gyroscope.
   *
   * @param gyroscopeCAN the Pigeon 2's CAN.
   */
  public GyroscopeIOPigeon2(CAN gyroscopeCAN) {
    pigeon2 = new Pigeon2(gyroscopeCAN.id(), gyroscopeCAN.bus());

    roll = pigeon2.getRoll();
    pitch = pigeon2.getRoll();
    yaw = pigeon2.getYaw();
  }

  @Override
  public void configure() {
    Pigeon2Configuration config = OdometryFactory.createGyroscopeConfig();

    ConfigApplier.applyPigeon2Config(pigeon2.getConfigurator(), config);
  }

  @Override
  public void update(GyroscopeIOValues values) {
    roll.refresh();
    pitch.refresh();
    yaw.refresh();

    values.rollRotations = Units.degreesToRotations(roll.getValue());
    values.pitchRotations = Units.degreesToRotations(pitch.getValue());
    values.yawRotations = Units.degreesToRotations(yaw.getValue());
  }

  @Override
  public void setYaw(double yawRotations) {
    pigeon2.setYaw(Units.rotationsToDegrees(yawRotations));
  }
}
