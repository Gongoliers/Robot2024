package frc.robot.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.superstructure.SuperstructureConstants.ShoulderAngleConstants;

public class ManualCommand extends Command {

    private final Arm arm;

    private final DoubleSupplier joystick;

    public ManualCommand(DoubleSupplier joystick) {
        arm = Arm.getInstance();

        this.joystick = joystick;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        double voltageScalar = 6.0;

        double volts = -joystick.getAsDouble() * voltageScalar;

        double positionRotations = arm.getMeasuredShoulderState().position;

        if (positionRotations > ShoulderAngleConstants.AMP.getRotations() && volts > 0) {
            volts = 0;
        }

        if (positionRotations < ShoulderAngleConstants.STOW.getRotations() && volts < 0) {
            volts = 0;
        }

        volts = MathUtil.clamp(volts, -12.0, voltageScalar);

        arm.setVoltage(volts);
    }

    @Override
    public void end(boolean interrupted) {
        arm.setVoltage(0);
    }
    
}
