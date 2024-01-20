package frc.robot.swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.lib.PIDFConstants;

/** Utility class for calculating PIDF for steer motors. */
public class SteerMotorPIDF {

  /** Feedback controller for position. */
  private final ProfiledPIDController positionFeedback;

  /** Feedforward controller for position. */
  private final SimpleMotorFeedforward positionFeedforward;

  /** Creates a PIDF utility class. */
  public SteerMotorPIDF(PIDFConstants pidfConstants) {
    positionFeedback =
        new ProfiledPIDController(
            pidfConstants.kP,
            pidfConstants.kI,
            pidfConstants.kD,
            new Constraints(
                pidfConstants.kVelocityConstraint, pidfConstants.kAccelerationConstraint));

    positionFeedback.enableContinuousInput(-0.5, 0.5);
    positionFeedback.setTolerance(pidfConstants.kPositionTolerance);

    positionFeedforward = new SimpleMotorFeedforward(pidfConstants.kS, pidfConstants.kV);
  }

  /**
   * Returns true if feedback error is within tolerance.
   *
   * @return true if feedback error is within tolerance.
   */
  public boolean atGoal() {
    return positionFeedback.atGoal();
  }

  /**
   * Calculates the voltage to reach a goal position from a measured position.
   *
   * @param measurement the measured position of the steer motor.
   * @param goal the goal position to reach.
   * @return the voltage to apply to the steer motor.
   */
  public double calculate(Rotation2d measurement, Rotation2d goal) {
    double positionFeedbackVolts =
        positionFeedback.calculate(measurement.getRotations(), goal.getRotations());

    double positionFeedforwardVolts =
        positionFeedforward.calculate(positionFeedback.getSetpoint().velocity);

    return positionFeedbackVolts + positionFeedforwardVolts;
  }
}
