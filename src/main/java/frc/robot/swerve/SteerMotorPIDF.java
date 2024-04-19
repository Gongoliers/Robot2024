package frc.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.PIDFConstants;

/** Utility class for calculating PIDF for steer motors. */
public class SteerMotorPIDF {

  /** Feedback controller for position. */
  private final PIDController feedback;

  private PIDFConstants pidfConstants;

  /** Creates a PIDF utility class. */
  public SteerMotorPIDF(PIDFConstants pidfConstants) {
    feedback = new PIDController(pidfConstants.kP, 0.0, pidfConstants.kD);

    feedback.enableContinuousInput(-0.5, 0.5);
    feedback.setTolerance(pidfConstants.kPositionTolerance);

    this.pidfConstants = pidfConstants;
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

    double feedforwardVolts = 0.0;

    if (feedback.atSetpoint() == false) {
      if (measurement.getRadians() > setpoint.getRadians()) {
        feedforwardVolts = pidfConstants.kS;
      } else {
        feedforwardVolts = -pidfConstants.kS;
      }
    }

    return feedbackVolts + feedforwardVolts;
  }
}
