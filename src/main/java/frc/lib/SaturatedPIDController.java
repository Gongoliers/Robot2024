package frc.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class SaturatedPIDController extends PIDController {
  // https://www.dmi.unict.it/santoro/teaching/sr/slides/PIDSaturation.pdf

  private boolean hasRange = false;
  private double min, max;

  public SaturatedPIDController(double kp, double ki, double kd) {
    super(kp, ki, kd);
  }

  public void setSaturation(double range) {
    setSaturation(-range, range);
  }

  private void setSaturation(double min, double max) {
    this.min = min;
    this.max = max;

    hasRange = true;
  }

  private double saturate(double calculated) {
    double clamped = calculated;

    if (hasRange) {
      clamped = MathUtil.clamp(calculated, min, max);
    }

    return clamped;
  }

  @Override
  public double calculate(double measurement) {
    double calculated = super.calculate(measurement);

    return saturate(calculated);
  }

  @Override
  public double calculate(double measurement, double setpoint) {
    double calculated = super.calculate(measurement, setpoint);

    return saturate(calculated);
  }
}
