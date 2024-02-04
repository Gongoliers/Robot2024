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

  /** Feedback controller for TalonFX velocity. */
  private final PIDController velocityFeedback =
      new PIDController(SwerveConstants.DRIVE_PIDF_CONSTANTS.kP, 0, 0);

  /** Feedforward controller for TalonFX velocity. */
  private final SimpleMotorFeedforward velocityFeedforward =
      new SimpleMotorFeedforward(
          SwerveConstants.DRIVE_PIDF_CONSTANTS.kS, SwerveConstants.DRIVE_PIDF_CONSTANTS.kV);

  /**
   * Creates a new TalonFX drive motor controlled by an external PID controller.
   *
   * @param can the TalonFX's CAN identifier.
   */
  public DriveMotorIOTalonFXPID(CAN can) {
    super(can);
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
      talonFX.setControl(calculateVelocityVoltage(velocityMetersPerSecond, false));
    }
  }

  /**
   * Calculates the TalonFX's applied voltage for a velocity setpoint.
   *
   * @param velocityMetersPerSecond the velocity setpoint.
   * @param isOpenLoop if true, uses open-loop control.
   * @return the voltage to apply.
   */
  private VoltageOut calculateVelocityVoltage(double velocityMetersPerSecond, boolean isOpenLoop) {
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
  private VoltageOut calculateOpenLoopVelocityVoltage(double velocityMetersPerSecond) {
    double velocityPercent = velocityMetersPerSecond / SwerveConstants.MAXIMUM_ATTAINABLE_SPEED;

    double velocityVolts = velocityPercent * RobotConstants.BATTERY_VOLTAGE;

    return new VoltageOut(velocityVolts);
  }

  /**
   * Calculates the TalonFX's applied voltage for a velocity setpoint using closed-loop.
   *
   * @param velocityMetersPerSecond the velocity setpoint.
   * @return the voltage to apply.
   */
  private VoltageOut calculateClosedLoopVelocityVoltage(double velocityMetersPerSecond) {
    double measuredVelocityMetersPerSecond =
        velocityRotationsPerSecond.getValue() * MK4iConstants.WHEEL_CIRCUMFERENCE;

    double velocityFeedbackVolts =
        velocityFeedback.calculate(measuredVelocityMetersPerSecond, velocityMetersPerSecond);

    double velocityFeedforwardVolts = velocityFeedforward.calculate(velocityMetersPerSecond);

    return new VoltageOut(velocityFeedbackVolts + velocityFeedforwardVolts);
  }
}
