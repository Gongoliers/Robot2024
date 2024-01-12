package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;

public class RotationPIDController extends SaturatedPIDController {

  public RotationPIDController(double kp, double ki, double kd) {
    super(kp, ki, kd);

    this.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void setSaturation(Rotation2d omega) {
    this.setSaturation(omega.getRadians());
  }

  public double calculate(Rotation2d rotation) {
    return calculate(rotation.getRadians());
  }

  public double calculate(Rotation2d rotation, Rotation2d setpoint) {
    return calculate(rotation.getRadians(), setpoint.getRadians());
  }
}
