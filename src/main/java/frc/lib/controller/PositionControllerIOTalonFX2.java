package frc.lib.controller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.lib.CAN;
import frc.lib.Configurator;
import frc.lib.ControllerConstants;

/** Position controller using two TalonFXs and a CANcoder and an external PIDF for an arm. */
public class PositionControllerIOTalonFX2 implements PositionControllerIO {

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
   * @param pidf
   * @param enableFOC
   * @param invertFollower
   */
  public PositionControllerIOTalonFX2(
      CAN leaderCAN,
      CAN followerCAN,
      CAN encoderCAN,
      ControllerConstants pidf,
      boolean enableFOC,
      boolean invertFollower) {
    leaderMotor = new TalonFX(leaderCAN.id(), leaderCAN.bus());
    followerMotor = new TalonFX(followerCAN.id(), followerCAN.bus());

    encoder = new CANcoder(encoderCAN.id(), encoderCAN.bus());

    position = encoder.getAbsolutePosition();

    velocity = leaderMotor.getVelocity();
    acceleration = leaderMotor.getAcceleration();

    volts = leaderMotor.getMotorVoltage();
    amps = leaderMotor.getStatorCurrent();

    feedforward = pidf.feedforward.createArmFeedforward();

    feedback = pidf.feedback.createPIDController();

    followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), invertFollower));

    voltage = new VoltageOut(0.0).withEnableFOC(enableFOC);
  }

  @Override
  public void configure(PositionControllerIOConstants constants) {
    BaseStatusSignal.setUpdateFrequencyForAll(100.0, position, velocity, acceleration, volts, amps);

    ParentDevice.optimizeBusUtilizationForAll(leaderMotor, followerMotor, encoder);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    motorConfig.MotorOutput.Inverted =
        constants.ccwPositiveMotor
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;
    motorConfig.MotorOutput.NeutralMode =
        constants.neutralBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    // Stator current is a measure of the current inside of the motor and is typically higher than
    // supply (breaker) current
    motorConfig.CurrentLimits.StatorCurrentLimit = constants.currentLimitAmps * 2.0;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    motorConfig.CurrentLimits.SupplyCurrentLimit = constants.currentLimitAmps;
    // Allow higher current spikes (150%) for a brief duration (one second)
    // REV 40A auto-resetting breakers typically trip when current exceeds 300% for one second
    motorConfig.CurrentLimits.SupplyCurrentThreshold = constants.currentLimitAmps * 1.5;
    motorConfig.CurrentLimits.SupplyTimeThreshold = 1;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    motorConfig.Feedback.SensorToMechanismRatio = constants.sensorToMechanismRatio;

    Configurator.configureTalonFX(leaderMotor.getConfigurator(), motorConfig);
    Configurator.configureTalonFX(followerMotor.getConfigurator(), motorConfig);

    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

    encoderConfig.MagnetSensor.MagnetOffset = constants.absoluteEncoderOffsetRotations;
    encoderConfig.MagnetSensor.SensorDirection =
        constants.ccwPositiveAbsoluteEncoder
            ? SensorDirectionValue.CounterClockwise_Positive
            : SensorDirectionValue.Clockwise_Positive;

    Configurator.configureCANcoder(encoder.getConfigurator(), encoderConfig);
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
