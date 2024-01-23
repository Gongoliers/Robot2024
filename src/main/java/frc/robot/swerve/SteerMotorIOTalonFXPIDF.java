package frc.robot.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.CAN;
import frc.lib.Configurator;
import frc.robot.swerve.SwerveConstants.MK4iConstants;

/** TalonFX steer motor controlled by an external PIDF controller. */
public class SteerMotorIOTalonFXPIDF extends SteerMotorIOTalonFX {

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

    pidf = new SteerMotorPIDF(SwerveConstants.STEER_PIDF_CONSTANTS);
  }

  @Override
  public void configure() {
    TalonFXConfiguration config = SwerveFactory.createSteerMotorConfig();

    config.Feedback.SensorToMechanismRatio = MK4iConstants.STEER_GEARING;

    Configurator.configureTalonFX(talonFX.getConfigurator(), config);
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
