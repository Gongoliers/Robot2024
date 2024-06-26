package frc.lib.controller;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.lib.CAN;
import frc.lib.config.MechanismConfig;

/** Velocity controller using TalonFX and external PIDF. */
public class VelocityControllerIOTalonFXPIDF extends VelocityControllerIOTalonFX {

  private final SimpleMotorFeedforward feedforward;

  private final PIDController feedback;

  private final VoltageOut voltage;

  /**
   * Creates a new velocity controller using TalonFX and external PIDF.
   *
   * @param can
   * @param config
   * @param enableFOC
   */
  public VelocityControllerIOTalonFXPIDF(CAN can, MechanismConfig config, boolean enableFOC) {
    super(can, config);

    feedforward = config.feedforwardControllerConfig().createSimpleMotorFeedforward();

    feedback = config.feedbackControllerConfig().createPIDController();

    voltage = new VoltageOut(0.0).withEnableFOC(enableFOC);
  }

  /**
   * Creates a new velocity controller using TalonFX and external PIDF.
   *
   * @param can
   * @param pidf
   */
  public VelocityControllerIOTalonFXPIDF(CAN can, MechanismConfig pidf) {
    this(can, pidf, false);
  }

  @Override
  public void setSetpoint(double velocityRotationsPerSecond) {
    double feedforwardVolts = feedforward.calculate(velocityRotationsPerSecond);

    double measuredVelocityRotationsPerSecond = velocity.getValue();

    double feedbackVolts =
        feedback.calculate(measuredVelocityRotationsPerSecond, velocityRotationsPerSecond);

    motor.setControl(voltage.withOutput(feedforwardVolts + feedbackVolts));
  }
}
