package frc.robot.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.CAN;
import frc.lib.MotorCurrentLimits;
import frc.robot.swerve.SwerveConstants.MK4iConstants;

/** TalonFX drive motor. */
public abstract class DriveMotorIOTalonFX implements DriveMotorIO {

  /** Hardware TalonFX. */
  protected final TalonFX talonFX;

  /** Hardware TalonFX base config. */
  protected final TalonFXConfiguration talonFXBaseConfig;

  /** TalonFX's position and velocity status signals. */
  protected final StatusSignal<Double> positionRotations, velocityRotationsPerSecond;

  /**
   * Creates a new TalonFX drive motor.
   *
   * @param driveMotorCAN the TalonFX's CAN identifier.
   */
  public DriveMotorIOTalonFX(CAN driveMotorCAN) {
    talonFX = new TalonFX(driveMotorCAN.id(), driveMotorCAN.bus());
    talonFXBaseConfig = new TalonFXConfiguration();

    talonFXBaseConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    talonFXBaseConfig.Feedback.SensorToMechanismRatio = MK4iConstants.DRIVE_GEARING;

    talonFXBaseConfig.CurrentLimits =
        new MotorCurrentLimits(60.0, 30.0, 60.0, 0.1).asCurrentLimitsConfigs();

    positionRotations = talonFX.getPosition();
    velocityRotationsPerSecond = talonFX.getVelocity();
  }

  @Override
  public abstract void configure();

  @Override
  public void update(DriveMotorIOValues values) {
    positionRotations.refresh();
    velocityRotationsPerSecond.refresh();

    values.positionMeters =
        BaseStatusSignal.getLatencyCompensatedValue(positionRotations, velocityRotationsPerSecond)
            * MK4iConstants.WHEEL_CIRCUMFERENCE;
    values.velocityMetersPerSecond =
        velocityRotationsPerSecond.getValue() * MK4iConstants.WHEEL_CIRCUMFERENCE;
  }

  @Override
  public void setPosition(double positionMeters) {
    talonFX.setPosition(positionMeters / MK4iConstants.WHEEL_CIRCUMFERENCE);
  }

  @Override
  public abstract void setSetpoint(double velocityMetersPerSecond);

  @Override
  public void setBrake(boolean brake) {
    if (brake) {
      talonFX.setNeutralMode(NeutralModeValue.Brake);
    } else {
      talonFX.setNeutralMode(NeutralModeValue.Coast);
    }
  }
}
