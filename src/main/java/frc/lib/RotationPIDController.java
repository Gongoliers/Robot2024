package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;

public class RotationPIDController extends SaturatedPIDController {

  public RotationPIDController(double kp, double ki, double kd) {
    super(kp, ki, kd);

    this.enableContinuousInput(-Math.PI, Math.PI);
  }

  public RotationPIDController withSaturation(Rotation2d saturation) {
    this.setSaturation(saturation.getRadians());

    return this;
  }

  public Rotation2d calculate(Rotation2d rotation) {
    return Rotation2d.fromRadians(calculate(rotation.getRadians()));
  }

  public Rotation2d calculate(Rotation2d rotation, Rotation2d setpoint) {
    return Rotation2d.fromRadians(calculate(rotation.getRadians(), setpoint.getRadians()));
  }
}
