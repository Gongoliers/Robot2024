package frc.lib;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ProfiledRotationPIDController extends ProfiledPIDController {

  public ProfiledRotationPIDController(double kp, double ki, double kd, Constraints constraints) {
    super(kp, ki, kd, constraints);

    this.enableContinuousInput(-0.5, 0.5);
  }

  public Rotation2d calculate(Rotation2d measurement) {
    double velocityRotationsPerSecond = calculate(measurement.getRotations());

    return Rotation2d.fromRotations(velocityRotationsPerSecond);
  }

  public Rotation2d calculate(Rotation2d measurement, Rotation2d goal) {
    double velocityRotationsPerSecond = calculate(measurement.getRotations(), goal.getRotations());

    return Rotation2d.fromRotations(velocityRotationsPerSecond);
  }
}
