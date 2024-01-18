package frc.robot.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.lib.CAN;
import frc.lib.ConfigApplier;
import frc.robot.swerve.SwerveConstants.MK4iConstants;
import frc.robot.swerve.SwerveConstants.SteerMotorConstants;

/** TalonFX steer motor controlled by an external PID controller. */
public class SteerMotorIOTalonFXPID extends SteerMotorIOTalonFX {

  /** Feedback controller for TalonFX position. */
  private final ProfiledPIDController positionFeedback =
      new ProfiledPIDController(
          SteerMotorConstants.FEEDBACK_KP,
          0,
          SteerMotorConstants.FEEDBACK_KD,
          new Constraints(
              SteerMotorConstants.MAXIMUM_SPEED.getRotations(),
              SteerMotorConstants.MAXIMUM_ACCELERATION.getRotations()));

  private final SimpleMotorFeedforward positionFeedforward =
      new SimpleMotorFeedforward(
          SteerMotorConstants.FEEDFORWARD_KS, SteerMotorConstants.FEEDFORWARD_KV);

  /**
   * Creates a new TalonFX steer motor controlled by an external PID controller.
   *
   * @param steerMotorCAN the TalonFX's CAN identifier.
   * @param azimuthEncoderCAN the CAN identifier for the steer motor's corresponding azimuth
   *     encoder.
   */
  public SteerMotorIOTalonFXPID(CAN steerMotorCAN, CAN azimuthEncoderCAN) {
    super(steerMotorCAN, azimuthEncoderCAN);

    positionFeedback.enableContinuousInput(-0.5, 0.5);
    positionFeedback.setTolerance(SteerMotorConstants.FEEDBACK_TOLERANCE.getRotations());
  }

  @Override
  public void configure() {
    TalonFXConfiguration config = SwerveFactory.createSteerMotorConfig();

    config.Feedback.SensorToMechanismRatio = MK4iConstants.STEER_GEARING;

    ConfigApplier.applyTalonFXConfig(talonFX.getConfigurator(), config);
  }

  @Override
  public void setSetpoint(double positionRotations) {
    positionFeedback.setGoal(positionRotations);

    if (positionFeedback.atSetpoint()) {
      talonFX.setControl(new CoastOut());
    } else {
      talonFX.setControl(new VoltageOut(calculatePositionVoltage(positionRotations)));
    }
  }

  /**
   * Calculates the TalonFX's applied voltage for a position setpoint.
   *
   * @param positionRotations the steer motor's setpoint.
   * @return the voltage to apply.
   */
  private double calculatePositionVoltage(double positionRotations) {
    double measuredPositionRotations =
        BaseStatusSignal.getLatencyCompensatedValue(
            this.positionRotations, this.velocityRotationsPerSecond);

    double positionFeedbackVolts =
        positionFeedback.calculate(measuredPositionRotations, positionRotations);

    double positionFeedforwardVolts =
        positionFeedforward.calculate(positionFeedback.getSetpoint().velocity);

    return positionFeedbackVolts + positionFeedforwardVolts;
  }
}
