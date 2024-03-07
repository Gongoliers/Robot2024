package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;

public class SingleJointedArmFeedforward {

  public static class SingleJointedArmFeedforwardConstants {
    public double kS;
    public double kG;
    public double kV;
    public double kA;

    public SingleJointedArmFeedforwardConstants() {
      this.kS = 0.0;
      this.kG = 0.0;
      this.kV = 0.0;
      this.kA = 0.0;
    }

    public SingleJointedArmFeedforwardConstants withKs(double kS) {
      this.kS = kS;
      return this;
    }

    public SingleJointedArmFeedforwardConstants withKg(double kG) {
      this.kG = kG;
      return this;
    }

    /**
     * Calculates the gravity compensation constant for an arm given the voltage applied at an
     * equilibrium position.
     *
     * @param angle the equilibrium position of the arm.
     * @param volts the voltage applied to the arm to hold it the equilibrium position.
     * @return the gravity compensation constant for the arm.
     */
    public SingleJointedArmFeedforwardConstants withKg(Rotation2d angle, double volts) {
      double kG = volts / angle.getCos();

      return this.withKg(kG);
    }

    public SingleJointedArmFeedforwardConstants withKv(double kV) {
      this.kV = kV;
      return this;
    }

    public SingleJointedArmFeedforwardConstants withKa(double kA) {
      this.kA = kA;
      return this;
    }
  }

  private final SingleJointedArmFeedforwardConstants constants;

  public SingleJointedArmFeedforward() {
    this.constants = new SingleJointedArmFeedforwardConstants();
  }

  public SingleJointedArmFeedforward(SingleJointedArmFeedforwardConstants constants) {
    this.constants = constants;
  }

  public double calculate(Rotation2d position) {
    return calculate(position, new Rotation2d());
  }

  public double calculate(Rotation2d position, Rotation2d velocity) {
    return calculate(position, new Rotation2d(), new Rotation2d());
  }

  public double calculate(Rotation2d position, Rotation2d velocity, Rotation2d acceleration) {
    return constants.kS * Math.signum(velocity.getRotations())
        + constants.kG * position.getCos()
        + constants.kV * velocity.getRotations()
        + constants.kA * acceleration.getRotations();
  }
}
