package frc.robot.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.lib.CAN;
import frc.lib.ConfigApplier;
import frc.robot.swerve.SwerveConstants.MK4iConstants;

/** TalonFX drive motor controlled by an external PID controller. */
public class DriveMotorIOTalonFXPID extends DriveMotorIOTalonFX {

  /** Feedback controller for TalonFX velocity. */
  private final PIDController velocityFeedback = new PIDController(1, 0, 0);

  /** Feedforward controller for TalonFX velocity. */
  private final SimpleMotorFeedforward velocityFeedforward = new SimpleMotorFeedforward(0.139, 0);

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
      talonFX.setControl(new VoltageOut(velocityMetersPerSecond / SwerveConstants.MAXIMUM_SPEED * 12));
    }
  }

  /**
   * Calculates the TalonFX's applied voltage for a velocity setpoint.
   *
   * @param velocityMetersPerSecond the velocity setpoint.
   * @return the voltage to apply.
   */
  private VoltageOut calculateVelocityVoltage(double velocityMetersPerSecond) {
    double measuredVelocityMetersPerSecond =
        velocityRotationsPerSecond.getValue() / MK4iConstants.WHEEL_CIRCUMFERENCE;

    double velocityFeedbackVolts =
        velocityFeedback.calculate(measuredVelocityMetersPerSecond, velocityMetersPerSecond);

    double velocityFeedforwardVolts = velocityFeedforward.calculate(velocityMetersPerSecond);

    return new VoltageOut(velocityFeedbackVolts + velocityFeedforwardVolts);
  }
}
