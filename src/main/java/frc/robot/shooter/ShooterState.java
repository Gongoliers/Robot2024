package frc.robot.shooter;

import java.util.Objects;

import edu.wpi.first.math.MathUtil;
import frc.robot.shooter.ShooterConstants.FlywheelConstants;
import frc.robot.shooter.ShooterConstants.SerializerConstants;

public record ShooterState(double flywheelVelocityRotationsPerSecond, double serializerVelocityRotationsPerSecond) {

    public ShooterState {
        Objects.requireNonNull(flywheelVelocityRotationsPerSecond);
        Objects.requireNonNull(serializerVelocityRotationsPerSecond);
    }

    public boolean at(ShooterState other) {
        return MathUtil.isNear(flywheelVelocityRotationsPerSecond, other.flywheelVelocityRotationsPerSecond, FlywheelConstants.SPEED_TOLERANCE) && MathUtil.isNear(serializerVelocityRotationsPerSecond, other.serializerVelocityRotationsPerSecond, SerializerConstants.SPEED_TOLERANCE);
    }
    
}
