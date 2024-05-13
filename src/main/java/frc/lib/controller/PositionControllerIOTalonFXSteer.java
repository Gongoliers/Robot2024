package frc.lib.controller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.lib.CAN;
import frc.lib.config.MechanismConfig;
import frc.lib.config.applier.CANcoderConfigApplier;
import frc.lib.config.applier.TalonFXConfigApplier;

/** Creates a new position controller using a steer TalonFX and azimuth CANcoder. */
public class PositionControllerIOTalonFXSteer implements PositionControllerIO {

  private final MechanismConfig config;

  private final TalonFX steerMotor;

  private final CANcoder azimuthEncoder;

  private final StatusSignal<Double> position, velocity, acceleration, volts, amps;

  private final SimpleMotorFeedforward feedforward;

  private final PIDController feedback;

  private final VoltageOut voltage;

  public PositionControllerIOTalonFXSteer(
      CAN steerCAN, CAN encoderCAN, MechanismConfig config, boolean enableFOC) {
    this.config = config;

    steerMotor = new TalonFX(steerCAN.id(), steerCAN.bus());

    azimuthEncoder = new CANcoder(encoderCAN.id(), encoderCAN.bus());

    // TODO Use steer position after seeding with azimuth to be more resistant to failure
    position = azimuthEncoder.getAbsolutePosition();

    velocity = steerMotor.getVelocity();
    acceleration = steerMotor.getAcceleration();

    volts = steerMotor.getMotorVoltage();
    amps = steerMotor.getStatorCurrent();

    feedforward = config.feedforwardControllerConfig().createSimpleMotorFeedforward();

    feedback = config.feedbackControllerConfig().createPIDController();

    voltage = new VoltageOut(0.0).withEnableFOC(enableFOC);
  }

  @Override
  public void configure() {
    BaseStatusSignal.setUpdateFrequencyForAll(100.0, position, velocity, acceleration, volts, amps);

    ParentDevice.optimizeBusUtilizationForAll(steerMotor, azimuthEncoder);

    TalonFXConfigApplier.apply(steerMotor, config.motorConfig());

    CANcoderConfigApplier.applyFactoryDefault(azimuthEncoder);
    CANcoderConfigApplier.apply(azimuthEncoder, config.absoluteEncoderConfig());
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
  public void setPosition(double positionRotations) {}

  @Override
  public void setSetpoint(double positionRotations, double velocityRotationsPerSecond) {
    double measuredPositionRotations = position.getValue();

    // double feedforwardVolts = feedforward.calculate(velocityRotationsPerSecond);
    double feedforwardVolts = calculateFeedforward(measuredPositionRotations, positionRotations);

    double feedbackVolts = feedback.calculate(measuredPositionRotations, positionRotations);

    steerMotor.setControl(voltage.withOutput(feedforwardVolts + feedbackVolts));
  }

  private double calculateFeedforward(double measurementRotations, double setpointRotations) {
    if (feedback.atSetpoint() == false) {
      if (measurementRotations > setpointRotations) {
        return feedforward.ks;
      } else {
        return -feedforward.ks;
      }
    }

    return 0.0;
  }
}
