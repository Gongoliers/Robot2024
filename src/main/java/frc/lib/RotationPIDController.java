package frc.lib;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class RotationPIDController extends PIDController {

  public RotationPIDController(double kp, double ki, double kd) {
    super(kp, ki, kd);

    this.enableContinuousInput(-0.5, 0.5);
  }

  public Rotation2d calculate(Rotation2d measurement) {
    double velocityRotationsPerSecond = calculate(measurement.getRadians());

    return Rotation2d.fromRotations(velocityRotationsPerSecond);
  }

  public Rotation2d calculate(Rotation2d measurement, Rotation2d goal) {
    double velocityRotationsPerSecond = calculate(measurement.getRadians(), goal.getRadians());

    return Rotation2d.fromRotations(velocityRotationsPerSecond);
  }
}
