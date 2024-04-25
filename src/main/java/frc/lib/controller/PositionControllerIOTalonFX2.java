package frc.lib.controller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.lib.CAN;
import frc.lib.config.Configurator;
import frc.lib.config.MechanismConfig;

/** Position controller using two TalonFXs and a CANcoder and an external PIDF for an arm. */
public class PositionControllerIOTalonFX2 implements PositionControllerIO {

  private final MechanismConfig config;

  private final TalonFX leaderMotor, followerMotor;

  private final CANcoder encoder;

  private final StatusSignal<Double> position, velocity, acceleration, volts, amps;

  private final ArmFeedforward feedforward;

  private final PIDController feedback;

  private final VoltageOut voltage;

  /**
   * Creates a new position controller using two TalonFXs and a CANcoder and an external PIDF for an
   * arm.
   *
   * @param leaderCAN
   * @param followerCAN
   * @param encoderCAN
   * @param config
   * @param enableFOC
   * @param invertFollower
   */
  public PositionControllerIOTalonFX2(
      CAN leaderCAN,
      CAN followerCAN,
      CAN encoderCAN,
      MechanismConfig config,
      boolean enableFOC,
      boolean invertFollower) {
    this.config = config;

    leaderMotor = new TalonFX(leaderCAN.id(), leaderCAN.bus());
    followerMotor = new TalonFX(followerCAN.id(), followerCAN.bus());

    encoder = new CANcoder(encoderCAN.id(), encoderCAN.bus());

    position = encoder.getAbsolutePosition();

    velocity = leaderMotor.getVelocity();
    acceleration = leaderMotor.getAcceleration();

    volts = leaderMotor.getMotorVoltage();
    amps = leaderMotor.getStatorCurrent();

    feedforward = config.feedforwardControllerConfig().createArmFeedforward();

    feedback = config.feedbackControllerConfig().createPIDController();

    followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), invertFollower));

    voltage = new VoltageOut(0.0).withEnableFOC(enableFOC);
  }

  @Override
  public void configure() {
    BaseStatusSignal.setUpdateFrequencyForAll(100.0, position, velocity, acceleration, volts, amps);

    ParentDevice.optimizeBusUtilizationForAll(leaderMotor, followerMotor, encoder);

    Configurator.configureTalonFX(
        leaderMotor.getConfigurator(), config.motorConfig().createTalonFXConfig());
    Configurator.configureTalonFX(
        followerMotor.getConfigurator(), config.motorConfig().createTalonFXConfig());

    Configurator.configureCANcoder(
        encoder.getConfigurator(), config.absoluteEncoderConfig().createCANcoderConfig());
  }

  @Override
  public void update(PositionControllerIOValues values) {
    BaseStatusSignal.refreshAll(position, velocity, acceleration, volts, amps);

    values.positionRotations = position.getValue();
    values.velocityRotationsPerSecond = velocity.getValue();
    values.accelerationRotationsPerSecondPerSecond = acceleration.getValue();
    values.motorVolts = volts.getValue();
    values.motorAmps = amps.getValue();
  }

  @Override
  public void setPosition(double positionRotations) {
    leaderMotor.setPosition(positionRotations);
    followerMotor.setPosition(positionRotations);
  }

  @Override
  public void setSetpoint(double positionRotations, double velocityRotationsPerSecond) {
    double measuredPositionRotations = position.getValue();

    double feedforwardVolts =
        feedforward.calculate(
            Units.rotationsToRadians(measuredPositionRotations), velocityRotationsPerSecond);

    double feedbackVolts = feedback.calculate(measuredPositionRotations, positionRotations);

    leaderMotor.setControl(voltage.withOutput(feedforwardVolts + feedbackVolts));
  }
}
