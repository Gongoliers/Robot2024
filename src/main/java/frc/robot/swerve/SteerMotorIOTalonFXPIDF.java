package frc.robot.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.CAN;
import frc.lib.ConfigApplier;
import frc.lib.PIDFConstants;
import frc.robot.swerve.SwerveConstants.MK4iConstants;

/** TalonFX steer motor controlled by an external PIDF controller. */
public class SteerMotorIOTalonFXPIDF extends SteerMotorIOTalonFX {

  /** Constants for PIDF position controller. */
  private static PIDFConstants pidfConstants = new PIDFConstants();

  static {
    pidfConstants.kP = 48.0; // volts per rotation
    pidfConstants.kD = 0.25; // volts per rotation per second
    pidfConstants.kPositionTolerance = Units.degreesToRotations(3);
    pidfConstants.kVelocityConstraint = 10.0; // rotations per second
    pidfConstants.kAccelerationConstraint = 64.0; // rotations per second per second
    pidfConstants.kS = 0.16; // volts
    pidfConstants.kV = 0.407363; // volts per rotation per second
  }

  /** PIDF position controller. */
  private final SteerMotorPIDF pidf;

  /**
   * Creates a new TalonFX steer motor controlled by an external PIDF controller.
   *
   * @param steerMotorCAN the TalonFX's CAN identifier.
   * @param azimuthEncoderCAN the CAN identifier for the steer motor's corresponding azimuth
   *     encoder.
   */
  public SteerMotorIOTalonFXPIDF(CAN steerMotorCAN, CAN azimuthEncoderCAN) {
    super(steerMotorCAN, azimuthEncoderCAN);

    pidf = new SteerMotorPIDF(pidfConstants);
  }

  @Override
  public void configure() {
    TalonFXConfiguration config = SwerveFactory.createSteerMotorConfig();

    config.Feedback.SensorToMechanismRatio = MK4iConstants.STEER_GEARING;

    ConfigApplier.applyTalonFXConfig(talonFX.getConfigurator(), config);
  }

  @Override
  public void setSetpoint(double positionRotations) {
    if (pidf.atGoal()) {
      // TODO Doesn't work for some reason...
      // talonFX.setControl(new CoastOut());
      // return;
    }

    Rotation2d measuredPosition =
        Rotation2d.fromRotations(
            BaseStatusSignal.getLatencyCompensatedValue(
                this.positionRotations, this.velocityRotationsPerSecond));

    double voltage = pidf.calculate(measuredPosition, Rotation2d.fromRotations(positionRotations));

    talonFX.setControl(new VoltageOut(voltage));
  }
}
