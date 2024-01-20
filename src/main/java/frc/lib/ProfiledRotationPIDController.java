package frc.lib;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ProfiledRotationPIDController extends ProfiledPIDController {

  public ProfiledRotationPIDController(double kp, double ki, double kd, Constraints constraints) {
    super(kp, ki, kd, constraints);

    this.enableContinuousInput(-0.5, 0.5);
  }

  public double calculate(Rotation2d measurement) {
    return calculate(measurement.getRotations());
  }

  public double calculate(Rotation2d measurement, Rotation2d goal) {
    return calculate(measurement.getRotations(), goal.getRotations());
  }
}
