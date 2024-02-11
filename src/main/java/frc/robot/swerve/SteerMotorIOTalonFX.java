package frc.robot.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.lib.CAN;
import frc.lib.MotorCurrentLimits;

/** TalonFX steer motor. */
public abstract class SteerMotorIOTalonFX implements SteerMotorIO {

  /** Hardware TalonFX. */
  protected final TalonFX talonFX;

  /** Hardware TalonFX base config. */
  protected final TalonFXConfiguration talonFXBaseConfig;

  /** TalonFX"s position and velocity status signals. */
  protected final StatusSignal<Double> positionRotations, velocityRotationsPerSecond;

  /** CAN identifier for the steer motor's corresponding azimuth encoder. */
  protected final CAN azimuthEncoderCAN;

  /**
   * Creates a new TalonFX steer motor.
   *
   * @param steerMotorCAN the TalonFX's CAN identifier.
   * @param azimuthEncoderCAN the CAN identifier for the steer motor's corresponding azimuth
   *     encoder.
   */
  public SteerMotorIOTalonFX(CAN steerMotorCAN, CAN azimuthEncoderCAN) {
    talonFX = new TalonFX(steerMotorCAN.id(), steerMotorCAN.bus());
    talonFXBaseConfig = new TalonFXConfiguration();

    talonFXBaseConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    talonFXBaseConfig.ClosedLoopGeneral.ContinuousWrap = true;

    // TODO Copied from Nutrons 2023 release
    talonFXBaseConfig.CurrentLimits =
        new MotorCurrentLimits(0.0, 40.0, 3.0, 1.0).asCurrentLimitsConfigs();

    positionRotations = talonFX.getPosition();
    velocityRotationsPerSecond = talonFX.getVelocity();

    this.azimuthEncoderCAN = azimuthEncoderCAN;
  }

  @Override
  public abstract void configure();

  @Override
  public void update(SteerMotorIOValues values) {
    positionRotations.refresh();
    velocityRotationsPerSecond.refresh();

    values.positionRotations =
        BaseStatusSignal.getLatencyCompensatedValue(positionRotations, velocityRotationsPerSecond);
    values.velocityRotationsPerSecond = velocityRotationsPerSecond.getValue();
  }

  @Override
  public void setPosition(double positionRotations) {
    talonFX.setPosition(positionRotations);
  }

  @Override
  public abstract void setSetpoint(double positionRotations);
}
