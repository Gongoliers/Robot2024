package frc.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;

public class LinearServo extends Servo {

  private final double speedMetersPerSecond;
  private final double maximumLengthMeters;

  private double position;
  private double setpointMeters;
  private double previousTimeSeconds = 0;

  public LinearServo(
      int channel, double maxiumumSpeedMetersPerSecond, double maximumLengthMetersPerSecond) {
    super(channel);

    this.speedMetersPerSecond = maxiumumSpeedMetersPerSecond;
    this.maximumLengthMeters = maximumLengthMetersPerSecond;

    setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
  }

  private void setBounds(
      double maxMilliseconds,
      double deadbandMaxMilliseconds,
      double centerMilliseconds,
      double deadbandMinMilliseconds,
      double minMilliseconds) {
    int maxMicroseconds = (int) (1000 * maxMilliseconds);
    int deadbandMaxMicroseconds = (int) (1000 * deadbandMaxMilliseconds);
    int centerMicroseconds = (int) (1000 * centerMilliseconds);
    int deadbandMinMicroseconds = (int) (1000 * deadbandMinMilliseconds);
    int minMicroseconds = (int) (1000 * minMilliseconds);

    setBoundsMicroseconds(
        maxMicroseconds,
        deadbandMaxMicroseconds,
        centerMicroseconds,
        deadbandMinMicroseconds,
        minMicroseconds);
  }

  public void setSetpoint(double setpointMeters) {
    setpointMeters = MathUtil.clamp(setpointMeters, 0, maximumLengthMeters);

    this.setpointMeters = setpointMeters;

    double percentage = setpointMeters / maximumLengthMeters;

    double speed = 2 * percentage - 1;

    setSpeed(speed);
  }

  public void updateCurPos() {
    double currentTimeSeconds = Timer.getFPGATimestamp();
    double dt = currentTimeSeconds - previousTimeSeconds;
    if (position > setpointMeters + speedMetersPerSecond * dt) {
      position -= speedMetersPerSecond * dt;
    } else if (position < setpointMeters - speedMetersPerSecond * dt) {
      position += speedMetersPerSecond * dt;
    } else {
      position = setpointMeters;
    }
  }
}
