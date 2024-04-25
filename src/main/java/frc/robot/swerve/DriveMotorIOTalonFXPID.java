package frc.robot.swerve;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.lib.CAN;
import frc.lib.Configurator;
import frc.robot.RobotConstants;
import frc.robot.swerve.SwerveConstants.MK4iConstants;

/** TalonFX drive motor controlled by an external PID controller. */
public class DriveMotorIOTalonFXPID extends DriveMotorIOTalonFX {

  /** Feedforward controller for TalonFX velocity. */
  private final SimpleMotorFeedforward velocityFeedforward;

  /** Feedback controller for TalonFX velocity. */
  private final PIDController velocityFeedback;

  /** Voltage output request. */
  private final VoltageOut voltageOutRequest;

  /**
   * Creates a new TalonFX drive motor controlled by an external PID controller.
   *
   * @param can the TalonFX's CAN identifier.
   */
  public DriveMotorIOTalonFXPID(CAN can) {
    super(can);

    velocityFeedforward = SwerveConstants.DRIVE_PIDF_CONSTANTS.feedforward.createSimpleMotorFeedforward();

    velocityFeedback = SwerveConstants.DRIVE_PIDF_CONSTANTS.feedback.createPIDController();

    voltageOutRequest = new VoltageOut(0).withEnableFOC(SwerveConstants.USE_PHOENIX_PRO_FOC);
  }

  @Override
  public void configure() {
    Configurator.configureTalonFX(talonFX.getConfigurator(), talonFXBaseConfig);
  }

  @Override
  public void setSetpoint(double velocityMetersPerSecond) {
    if (velocityMetersPerSecond == 0.0) {
      talonFX.setControl(new CoastOut());
    } else {
      double volts = calculateVelocityVoltage(velocityMetersPerSecond, false);

      talonFX.setControl(voltageOutRequest.withOutput(volts));
    }
  }

  /**
   * Calculates the TalonFX's applied voltage for a velocity setpoint.
   *
   * @param velocityMetersPerSecond the velocity setpoint.
   * @param isOpenLoop if true, uses open-loop control.
   * @return the voltage to apply.
   */
  private double calculateVelocityVoltage(double velocityMetersPerSecond, boolean isOpenLoop) {
    return isOpenLoop
        ? calculateOpenLoopVelocityVoltage(velocityMetersPerSecond)
        : calculateClosedLoopVelocityVoltage(velocityMetersPerSecond);
  }

  /**
   * Calculates the TalonFX's applied voltage for a velocity setpoint using open-loop.
   *
   * @param velocityMetersPerSecond the velocity setpoint.
   * @return the voltage to apply.
   */
  private double calculateOpenLoopVelocityVoltage(double velocityMetersPerSecond) {
    double velocityPercent = velocityMetersPerSecond / SwerveConstants.MAXIMUM_ATTAINABLE_SPEED;

    double velocityVolts = velocityPercent * RobotConstants.BATTERY_VOLTAGE;

    return velocityVolts;
  }

  /**
   * Calculates the TalonFX's applied voltage for a velocity setpoint using closed-loop.
   *
   * @param velocityMetersPerSecond the velocity setpoint.
   * @return the voltage to apply.
   */
  private double calculateClosedLoopVelocityVoltage(double velocityMetersPerSecond) {
    double measuredVelocityMetersPerSecond =
        velocityRotationsPerSecond.getValue() * MK4iConstants.WHEEL_CIRCUMFERENCE;

    double velocityFeedbackVolts =
        velocityFeedback.calculate(measuredVelocityMetersPerSecond, velocityMetersPerSecond);

    double velocityFeedforwardVolts = velocityFeedforward.calculate(velocityMetersPerSecond);

    return velocityFeedbackVolts + velocityFeedforwardVolts;
  }
}
