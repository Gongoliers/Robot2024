package frc.robot.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.lib.CAN;
import frc.lib.ConfigApplier;
import frc.robot.swerve.SwerveConstants.MK4iConstants;

/** TalonFX steer motor controlled by an external PID controller. */
public class SteerMotorIOTalonFXPID extends SteerMotorIOTalonFX {

  /** Feedback controller for TalonFX position. */
  private final PIDController positionFeedback = new PIDController(32, 0, 0.25);

  /** Static feedforward for TalonFX position in volts. */
  private final double positionFeedforwardVolts = 0.14;

  /**
   * Creates a new TalonFX steer motor controlled by an external PID controller.
   *
   * @param steerMotorCAN the TalonFX's CAN identifier.
   * @param azimuthEncoderCAN the CAN identifier for the steer motor's corresponding azimuth
   *     encoder.
   */
  public SteerMotorIOTalonFXPID(CAN steerMotorCAN, CAN azimuthEncoderCAN) {
    super(steerMotorCAN, azimuthEncoderCAN);

    positionFeedback.enableContinuousInput(0, 1);
    positionFeedback.setTolerance(Units.degreesToRotations(1));
  }

  @Override
  public void configure() {
    TalonFXConfiguration config = SwerveFactory.createSteerMotorConfig();

    config.Feedback.SensorToMechanismRatio = MK4iConstants.STEER_GEARING;

    ConfigApplier.applyTalonFXConfig(talonFX.getConfigurator(), config);
  }

  @Override
  public void setSetpoint(double positionRotations) {
    positionFeedback.setSetpoint(positionRotations);

    if (positionFeedback.atSetpoint()) {
      talonFX.setControl(new CoastOut());
    } else {
      talonFX.setControl(calculatePositionVoltage());
    }
  }

  /**
   * Calculates the TalonFX's applied voltage for a position setpoint.
   *
   * @return the voltage to apply.
   */
  private VoltageOut calculatePositionVoltage() {
    double measuredPositionRotations =
        BaseStatusSignal.getLatencyCompensatedValue(
            this.positionRotations, this.velocityRotationsPerSecond);

    double positionFeedbackVolts = positionFeedback.calculate(measuredPositionRotations);

    double positionFeedforwardVolts =
        Math.signum(positionFeedbackVolts) * this.positionFeedforwardVolts;

    return new VoltageOut(positionFeedbackVolts + positionFeedforwardVolts);
  }
}
