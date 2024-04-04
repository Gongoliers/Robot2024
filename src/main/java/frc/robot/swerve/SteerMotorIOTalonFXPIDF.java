package frc.robot.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.CAN;
import frc.lib.Configurator;
import frc.lib.MotorCurrentLimits;
import frc.robot.swerve.SwerveConstants.MK4iConstants;

/** TalonFX steer motor controlled by an external PIDF controller. */
public class SteerMotorIOTalonFXPIDF extends SteerMotorIOTalonFX {

  /** PIDF position controller. */
  private final SteerMotorPIDF pidf;

  /** Voltage output request. */
  private final VoltageOut voltageOutRequest;

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

    voltageOutRequest = new VoltageOut(0).withEnableFOC(SwerveConstants.USE_PHOENIX_PRO_FOC);
  }

  @Override
  public void configure() {
    TalonFXConfiguration talonFXPIDFConfig = new TalonFXConfiguration();

    talonFXPIDFConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    talonFXPIDFConfig.ClosedLoopGeneral.ContinuousWrap = true;

    talonFXPIDFConfig.CurrentLimits =
        new MotorCurrentLimits(35.0, 15.0, 25.0, 0.1).asCurrentLimitsConfigs();

    talonFXPIDFConfig.Feedback.SensorToMechanismRatio = MK4iConstants.STEER_GEARING;

    Configurator.configureTalonFX(talonFX.getConfigurator(), talonFXPIDFConfig);
  }

  @Override
  public void setSetpoint(double positionRotations) {
    Rotation2d measuredPosition =
        Rotation2d.fromRotations(
            BaseStatusSignal.getLatencyCompensatedValue(
                this.positionRotations, this.velocityRotationsPerSecond));

    double voltage = pidf.calculate(measuredPosition, Rotation2d.fromRotations(positionRotations));

    talonFX.setControl(voltageOutRequest.withOutput(voltage));
  }
}
