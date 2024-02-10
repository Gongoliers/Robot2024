package frc.robot.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import frc.lib.CAN;
import frc.lib.Configurator;
import frc.robot.swerve.SwerveConstants.MK4iConstants;

/** TalonFX steer motor controlled by an internal controller. */
public class SteerMotorIOTalonFXIntegrated extends SteerMotorIOTalonFX {

  private final MotionMagicVoltage motionMagicVoltageSetter = new MotionMagicVoltage(0);

  public SteerMotorIOTalonFXIntegrated(CAN steerMotorCAN, CAN azimuthEncoderCAN) {
    super(steerMotorCAN, azimuthEncoderCAN);
  }

  @Override
  public void configure() {
    TalonFXConfiguration talonFXIntegratedConfig = new TalonFXConfiguration();

    // TODO
    talonFXIntegratedConfig.deserialize(talonFXBaseConfig.serialize());

    talonFXIntegratedConfig.Feedback.SensorToMechanismRatio = MK4iConstants.STEER_GEARING;

    talonFXIntegratedConfig.Slot0 = SwerveConstants.STEER_PIDF_CONSTANTS.asSlot0Configs();

    // TODO Copied from CTRE swerve library
    // https://api.ctr-electronics.com/phoenix6/release/java/src-html/com/ctre/phoenix6/mechanisms/swerve/SwerveModule.html#line.123
    talonFXIntegratedConfig.MotionMagic.MotionMagicCruiseVelocity =
        100.0 / MK4iConstants.STEER_GEARING;
    talonFXIntegratedConfig.MotionMagic.MotionMagicAcceleration =
        talonFXIntegratedConfig.MotionMagic.MotionMagicCruiseVelocity * 10;

    Configurator.configureTalonFX(talonFX.getConfigurator(), talonFXIntegratedConfig);
  }

  @Override
  public void runSetpoint(double positionRotations) {
    talonFX.setControl(motionMagicVoltageSetter.withPosition(positionRotations));
  }
}
