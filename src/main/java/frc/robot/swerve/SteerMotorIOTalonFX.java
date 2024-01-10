package frc.robot.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.CAN;

/** TalonFX steer motor. */
public abstract class SteerMotorIOTalonFX implements SteerMotorIO {

  /** Hardware TalonFX. */
  protected final TalonFX talonFX;

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
