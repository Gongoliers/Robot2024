package frc.lib;

import edu.wpi.first.math.system.plant.DCMotor;
import java.util.Objects;

public record JointConstants(
    double mass,
    double length,
    double radius,
    double moi,
    double gearing,
    DCMotor motor,
    int motorCount) {

  /**
   * Creates joint constants for the joint of an arm.
   *
   * @param mass the joint's mass in kilograms.
   * @param length the joint's length in meters.
   * @param radius the distance between the joint's pivot and the joint's center of mass in meters.
   * @param moi the joint's moment of inertia in kilograms meters squared.
   * @param gearing the gearing between the joint's motor and the joint's pivot.
   * @param motor the type of the joint's driving motor.
   * @param motorCount the number of the driving motors.
   */
  public JointConstants {
    Objects.requireNonNull(mass);
    Objects.requireNonNull(length);
    Objects.requireNonNull(radius);
    Objects.requireNonNull(moi);
    Objects.requireNonNull(gearing);
    Objects.requireNonNull(motor);
    Objects.requireNonNull(motorCount);
  }

  /**
   * Creates joint constants for the joint of an arm.
   *
   * @param mass the joint's mass in kilograms.
   * @param length the joint's length in meters.
   * @param radius the distance between the joint's pivot and the joint's center of mass in meters.
   * @param moi the joint's moment of inertia in kilograms meters squared.
   * @param gearing the gearing between the joint's motor and the joint's pivot.
   * @param motor the type of the joint's driving motor.
   */
  public JointConstants(
      double mass, double length, double radius, double moi, double gearing, DCMotor motor) {
    this(mass, length, radius, moi, gearing, motor, 1);
  }

  /**
   * Calculates the torque generated by the joint's gearbox element for the feedforward matrix.
   *
   * @return the torque generated by the joint's gearbox element for the feedforward matrix.
   */
  public double torque() {
    return gearing * motorCount * motor.KtNMPerAmp / motor.rOhms;
  }

  /**
   * Calculates the torque loss due to back-emf for the feedforward matrix.
   *
   * @return the torque loss due to back-emf for the feedforward matrix.
   */
  public double torqueLoss() {
    return gearing
        * gearing
        * motorCount
        * motor.KtNMPerAmp
        / motor.rOhms
        / motor.KvRadPerSecPerVolt;
  }
}
