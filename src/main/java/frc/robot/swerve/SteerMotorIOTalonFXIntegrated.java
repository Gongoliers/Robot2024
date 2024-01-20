package frc.robot.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import frc.lib.CAN;
import frc.lib.ConfigApplier;
import frc.robot.swerve.SwerveConstants.MK4iConstants;

/** TalonFX steer motor controlled by an internal controller. */
public class SteerMotorIOTalonFXIntegrated extends SteerMotorIOTalonFX {

  private final MotionMagicVoltage motionMagicVoltageSetter = new MotionMagicVoltage(0);

  public SteerMotorIOTalonFXIntegrated(CAN steerMotorCAN, CAN azimuthEncoderCAN) {
    super(steerMotorCAN, azimuthEncoderCAN);
  }

  @Override
  public void configure() {
    TalonFXConfiguration config = SwerveFactory.createSteerMotorConfig();

    config.Feedback.SensorToMechanismRatio = MK4iConstants.STEER_GEARING;

    config.Slot0 = SwerveConstants.STEER_PIDF_CONSTANTS.asSlot0Configs();

    // TODO Copied from CTRE swerve library
    // https://api.ctr-electronics.com/phoenix6/release/java/src-html/com/ctre/phoenix6/mechanisms/swerve/SwerveModule.html#line.123
    config.MotionMagic.MotionMagicCruiseVelocity = 100.0 / MK4iConstants.STEER_GEARING;
    config.MotionMagic.MotionMagicAcceleration = config.MotionMagic.MotionMagicCruiseVelocity * 10;

    ConfigApplier.applyTalonFXConfig(talonFX.getConfigurator(), config);
  }

  @Override
  public void setSetpoint(double positionRotations) {
    talonFX.setControl(motionMagicVoltageSetter.withPosition(positionRotations));
  }
}
