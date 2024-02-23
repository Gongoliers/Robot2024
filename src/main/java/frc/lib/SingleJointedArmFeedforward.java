package frc.lib;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

public class SingleJointedArmFeedforward {

    private final ArmFeedforward feedforward;

    public SingleJointedArmFeedforward(double ks, double kg, double kv) {
        feedforward = new ArmFeedforward(ks, kg, kv);
    }

    public SingleJointedArmFeedforward(double ks, double kg, double kv, double ka) {
        feedforward = new ArmFeedforward(ks, kg, kv, ka);
    }

    public double calculate(Rotation2d position) {
        return feedforward.calculate(position.getRadians(), 0);
    }

    public double calculate(Rotation2d position, double velocity) {
        return feedforward.calculate(position.getRadians(), velocity);
    }

    public double calculate(Rotation2d position, double velocity, double acceleration) {
        return feedforward.calculate(position.getRadians(), velocity, acceleration);
    }

}
