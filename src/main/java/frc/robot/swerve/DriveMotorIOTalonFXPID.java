package frc.robot.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.lib.CAN;
import frc.lib.ConfigApplier;
import frc.robot.RobotConstants;
import frc.robot.swerve.SwerveConstants.DriveMotorConstants;
import frc.robot.swerve.SwerveConstants.MK4iConstants;

/** TalonFX drive motor controlled by an external PID controller. */
public class DriveMotorIOTalonFXPID extends DriveMotorIOTalonFX {

  /** Feedback controller for TalonFX velocity. */
  private final PIDController velocityFeedback = new PIDController(DriveMotorConstants.FEEDBACK_KP, 0, 0);

  /** Feedforward controller for TalonFX velocity. */
  private final SimpleMotorFeedforward velocityFeedforward = new SimpleMotorFeedforward(DriveMotorConstants.FEEDFORWARD_KS, 0);

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
    TalonFXConfiguration config = SwerveFactory.createDriveMotorConfig();

    ConfigApplier.applyTalonFXConfig(talonFX.getConfigurator(), config);
  }

  @Override
  public void setSetpoint(double velocityMetersPerSecond) {
    if (velocityMetersPerSecond == 0.0) {
      talonFX.setControl(new CoastOut());
    } else {
      talonFX.setControl(calculateVelocityVoltage(velocityMetersPerSecond, DriveMotorConstants.OPEN_LOOP));
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
        : calculateFeedbackVelocityVoltage(velocityMetersPerSecond);
  }

  /**
   * Calculates the TalonFX's applied voltage for a velocity setpoint using open-loop.
   *
   * @param velocityMetersPerSecond the velocity setpoint.
   * @return the voltage to apply.
   */
  private VoltageOut calculateOpenLoopVelocityVoltage(double velocityMetersPerSecond) {
    double velocityPercent = velocityMetersPerSecond / SwerveConstants.MAXIMUM_SPEED;

    double velocityVolts = velocityPercent * RobotConstants.BATTERY_VOLTAGE;

    return new VoltageOut(velocityVolts);
  }

  /**
   * Calculates the TalonFX's applied voltage for a velocity setpoint using feedback.
   *
   * @param velocityMetersPerSecond the velocity setpoint.
   * @return the voltage to apply.
   */
  private VoltageOut calculateFeedbackVelocityVoltage(double velocityMetersPerSecond) {
    double measuredVelocityMetersPerSecond =
        velocityRotationsPerSecond.getValue() / MK4iConstants.WHEEL_CIRCUMFERENCE;

    double velocityFeedbackVolts =
        velocityFeedback.calculate(measuredVelocityMetersPerSecond, velocityMetersPerSecond);

    double velocityFeedforwardVolts = velocityFeedforward.calculate(velocityMetersPerSecond);

    return new VoltageOut(velocityFeedbackVolts + velocityFeedforwardVolts);
  }
}
