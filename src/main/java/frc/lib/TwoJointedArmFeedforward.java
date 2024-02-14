package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import org.ejml.data.MatrixType;
import org.ejml.simple.SimpleMatrix;

public class TwoJointedArmFeedforward {

  private final double m1;
  private final double m2;
  private final double l1;
  private final double r1;
  private final double r2;
  private final double I1;
  private final double I2;
  private final double g1;
  private final double g2;

  private final SimpleMatrix M_MATRIX = new SimpleMatrix(2, 2, MatrixType.DDRM);
  private final SimpleMatrix C_MATRIX = new SimpleMatrix(2, 2, MatrixType.DDRM);
  private final SimpleMatrix Tg_VECTOR = new SimpleMatrix(2, 1, MatrixType.DDRM);
  private final SimpleMatrix Kb_MATRIX = new SimpleMatrix(2, 2, MatrixType.DDRM);
  private final SimpleMatrix B_MATRIX = new SimpleMatrix(2, 2, MatrixType.DDRM);

  /**
   * The constructor for the system model.
   *
   * @param systemConstants the system constants of the arm.
   */
  public TwoJointedArmFeedforward(SystemConstants systemConstants) {
    var shoulderJointConstants = systemConstants.getShoulderJointConstants();
    var elbowJointConstants = systemConstants.getElbowJointConstants();
    this.m1 = shoulderJointConstants.mass;
    this.m2 = elbowJointConstants.mass;
    this.l1 = shoulderJointConstants.length;
    this.r1 = shoulderJointConstants.cmRadius;
    this.r2 = elbowJointConstants.cmRadius;
    this.I1 = shoulderJointConstants.momentOfInertia;
    this.I2 = elbowJointConstants.momentOfInertia;
    this.g1 = shoulderJointConstants.gearing;
    this.g2 = elbowJointConstants.gearing;
    int n1 = shoulderJointConstants.numberOfMotors;
    int n2 = elbowJointConstants.numberOfMotors;

    // TODO
    var motor = DCMotor.getFalcon500(1);

    B_MATRIX.set(0, 0, g1 * n1 * motor.KtNMPerAmp / motor.rOhms);
    B_MATRIX.set(1, 1, g2 * n2 * motor.KtNMPerAmp / motor.rOhms);
    B_MATRIX.set(1, 0, 0);
    B_MATRIX.set(0, 1, 0);

    Kb_MATRIX.set(0, 0, g1 * g1 * n1 * motor.KtNMPerAmp / motor.rOhms / motor.KvRadPerSecPerVolt);
    Kb_MATRIX.set(1, 1, g2 * g2 * n2 * motor.KtNMPerAmp / motor.rOhms / motor.KvRadPerSecPerVolt);
    Kb_MATRIX.set(1, 0, 0);
    Kb_MATRIX.set(0, 1, 0);
  }

  /**
   * Calculates the M matrix.
   *
   * @param elbowPosition the position of the elbow joint.
   */
  public void updateMMatrix(Rotation2d elbowPosition) {
    double c2 = elbowPosition.getCos();
    double diagonal = m2 * (r2 * r2 + l1 * r2 * c2) + I2;

    M_MATRIX.set(0, 0, m1 * r1 * r1 + m2 * (l1 * l1 + r2 * r2 + 2 * l1 * r2 * c2) + I1 + I2);
    M_MATRIX.set(0, 1, diagonal);
    M_MATRIX.set(1, 0, diagonal);
    M_MATRIX.set(1, 1, m2 * r2 * r2 + I2);
  }

  /**
   * Calculates the C matrix.
   *
   * @param elbowPosition the position of the elbow joint.
   * @param shoulderVelocity the velocity of the shoulder joint.
   * @param elbowVelocity the velocity of the elbow joint.
   */
  public void updateCMatrix(
      Rotation2d elbowPosition, Rotation2d shoulderVelocity, Rotation2d elbowVelocity) {
    double s2 = elbowPosition.getSin();

    C_MATRIX.set(0, 0, -m2 * l1 * r2 * s2 * elbowVelocity.getRadians());
    C_MATRIX.set(
        0, 1, -m2 * l1 * r2 * s2 * (shoulderVelocity.getRadians() + elbowVelocity.getRadians()));
    C_MATRIX.set(1, 0, m2 * l1 * r2 * s2 * shoulderVelocity.getRadians());
    C_MATRIX.set(1, 1, 0);
  }

  /**
   * Calculates the Tg vector.
   *
   * @param shoulderPosition the position of the shoulder joint.
   * @param elbowPosition the position of the elbow joint.
   */
  public void updateTgVector(Rotation2d shoulderPosition, Rotation2d elbowPosition) {
    double c1 = shoulderPosition.getCos();
    double c12 = shoulderPosition.plus(elbowPosition).getCos();

    Tg_VECTOR.set(0, 0, (m1 * r1 + m2 * l1) * g1 * c1 + m2 * r2 * g2 * c12);
    Tg_VECTOR.set(1, 0, m2 * r2 * g2 * c12);
  }

  /**
   * Calculates the voltage feedforward required to move the arm in a certain state.
   *
   * @param shoulderPosition the angle of the shoulder joint.
   * @param elbowPosition the angle of the elbow joint.
   * @param shoulderVelocity the angular velocity of the shoulder joint.
   * @param elbowVelocity the angular velocity of the elbow joint.
   * @param shoulderAcceleration the angular acceleration of the shoulder joint.
   * @param elbowAcceleration the angular acceleration of the elbow joint.
   * @return the voltage feedforward required to move the arm in the given state.
   */
  public ArmFeedForward calculateFeedForward(
      Rotation2d shoulderPosition,
      Rotation2d elbowPosition,
      Rotation2d shoulderVelocity,
      Rotation2d elbowVelocity,
      Rotation2d shoulderAcceleration,
      Rotation2d elbowAcceleration) {
    updateMMatrix(elbowPosition);
    updateCMatrix(elbowPosition, shoulderVelocity, elbowVelocity);
    updateTgVector(shoulderPosition, elbowPosition);

    var thetaDotVector = new SimpleMatrix(2, 1, MatrixType.DDRM);
    thetaDotVector.set(0, 0, shoulderVelocity.getRadians());
    thetaDotVector.set(1, 0, elbowVelocity.getRadians());

    var thetaDotDotVector = new SimpleMatrix(2, 1, MatrixType.DDRM);
    thetaDotDotVector.set(0, 0, shoulderAcceleration.getRadians());
    thetaDotDotVector.set(1, 0, elbowAcceleration.getRadians());

    var M = M_MATRIX.mult(thetaDotDotVector);
    var C = C_MATRIX.mult(thetaDotVector);
    var Kb = Kb_MATRIX.mult(thetaDotVector);
    var B_INV = B_MATRIX.invert();

    var u = B_INV.mult(M.plus(C).plus(Kb).plus(Tg_VECTOR));

    return new ArmFeedForward(u.get(0, 0), u.get(1, 0));
  }

  /** Calculates the voltage feedforward required to move the arm in a certain state. */
  public static class ArmFeedForward {
    double shoulderFeedForward;
    double elbowFeedForward;

    /**
     * Constructor.
     *
     * @param shoulderFeedForward the voltage feedforward required to move the shoulder joint. [V]
     * @param elbowFeedForward the voltage feedforward required to move the elbow joint. [V]
     */
    public ArmFeedForward(double shoulderFeedForward, double elbowFeedForward) {
      this.shoulderFeedForward = shoulderFeedForward;
      this.elbowFeedForward = elbowFeedForward;
    }
  }
}
