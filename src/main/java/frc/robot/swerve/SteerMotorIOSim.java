package frc.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.RobotConstants;
import frc.robot.swerve.SwerveConstants.MK4iConstants;

/** Simulated steer motor. */
public class SteerMotorIOSim implements SteerMotorIO {

    /** Represents the motor used to steer the wheel. */
    private final DCMotor motorSim = DCMotor.getFalcon500(1); // TODO

    /** Represents the wheel steered by the motor. */
    private final FlywheelSim wheelSim = new FlywheelSim(motorSim, MK4iConstants.STEER_GEARING, MK4iConstants.STEER_MOI);

    /** Represents the angle of the steer motor. */
    private double angleRotations;

    /** Feedback controller for the angle. */
    private final PIDController angleController = new PIDController(1.0, 0, 0);

    public SteerMotorIOSim() {
        angleRotations = 0.0;

        angleController.enableContinuousInput(0, 1);
    }

    @Override
    public void update(SteerMotorIOValues values) {
        double voltage = angleController.calculate(angleRotations);

        wheelSim.setInputVoltage(voltage);
        wheelSim.update(RobotConstants.TICK_PERIOD);

        double omegaRotationsPerSecond = wheelSim.getAngularVelocityRPM() / 60.0;
        angleRotations += omegaRotationsPerSecond;

        values.angleRotations = angleRotations;
        values.omegaRotationsPerSecond = omegaRotationsPerSecond;
    }

    @Override
    public void setPosition(double angleRotations) {
        this.angleRotations = 0.0;
    }

    @Override
    public void setSetpoint(double angleRotations) {
        angleController.setSetpoint(angleRotations);
    }
    


}
