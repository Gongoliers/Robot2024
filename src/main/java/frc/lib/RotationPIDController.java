package frc.lib;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class RotationPIDController extends PIDController {

  public RotationPIDController(double kp, double ki, double kd) {
    super(kp, ki, kd);

    this.enableContinuousInput(-0.5, 0.5);
  }

  public double calculate(Rotation2d measurement) {
    return calculate(measurement.getRadians());
  }

  public double calculate(Rotation2d measurement, Rotation2d goal) {
    return calculate(measurement.getRadians(), goal.getRadians());
  }
}
