package frc.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.ControllerConstants;

/** Utility class for calculating PIDF for steer motors. */
public class SteerMotorPIDF {

  /** Feedforward controller for position. */
  private final SimpleMotorFeedforward feedforward;

  /** Feedback controller for position. */
  private final PIDController feedback;

  /** Creates a PIDF utility class. */
  public SteerMotorPIDF(ControllerConstants pidfConstants) {
    feedforward = pidfConstants.feedforward.createSimpleMotorFeedforward();

    feedback = pidfConstants.feedback.createPIDController();
    feedback.enableContinuousInput(-0.5, 0.5);
  }

  /**
   * Returns true if feedback error is within tolerance.
   *
   * @return true if feedback error is within tolerance.
   */
  public boolean atSetpoint() {
    return feedback.atSetpoint();
  }

  /**
   * Calculates the voltage to reach a setpoint position from a measured position.
   *
   * @param measurement the measured position of the steer motor.
   * @param setpoint the setpoint position.
   * @return the voltage to apply to the steer motor.
   */
  public double calculate(Rotation2d measurement, Rotation2d setpoint) {
    double feedbackVolts = feedback.calculate(measurement.getRotations(), setpoint.getRotations());

    double feedforwardVolts = feedforward.calculate(0.0);

    return feedbackVolts + feedforwardVolts;
  }
}
