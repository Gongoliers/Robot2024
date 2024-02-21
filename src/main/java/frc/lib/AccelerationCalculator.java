package frc.lib;

import edu.wpi.first.wpilibj.Timer;

public class AccelerationCalculator {

  private double previousVelocity;
  private double previousTime;

  public double calculate(double velocity) {
    double time = Timer.getFPGATimestamp();

    double acceleration = (velocity - previousVelocity) / (time - previousTime);

    previousVelocity = velocity;
    previousTime = time;

    return acceleration;
  }
}
