package frc.lib;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class RotationPIDController extends PIDController {

  public RotationPIDController(double kp, double ki, double kd) {
    super(kp, ki, kd);

    this.enableContinuousInput(-Math.PI, Math.PI);
  }

  public Rotation2d calculate(Rotation2d rotation) {
    return Rotation2d.fromRadians(calculate(rotation.getRadians()));
  }

  public Rotation2d calculate(Rotation2d rotation, Rotation2d setpoint) {
    return Rotation2d.fromRadians(calculate(rotation.getRadians(), setpoint.getRadians()));
  }
}
