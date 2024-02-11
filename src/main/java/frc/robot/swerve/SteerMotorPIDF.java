package frc.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.PIDFConstants;

/** Utility class for calculating PIDF for steer motors. */
public class SteerMotorPIDF {

  /** Feedback controller for position. */
  private final PIDController feedback;

  /** Feedforward controller for position. */
  private final SimpleMotorFeedforward feedforward;

  /** Creates a PIDF utility class. */
  public SteerMotorPIDF(PIDFConstants pidfConstants) {
    feedback = new PIDController(pidfConstants.kP, 0.0, pidfConstants.kD);

    feedback.enableContinuousInput(-0.5, 0.5);
    feedback.setTolerance(pidfConstants.kPositionTolerance);

    feedforward = new SimpleMotorFeedforward(pidfConstants.kS, 0.0);
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

    // TODO Determine if/how velocity feedforward could be used
    double feedforwardVolts = feedforward.calculate(0.0);

    return feedbackVolts + feedforwardVolts;
  }
}
