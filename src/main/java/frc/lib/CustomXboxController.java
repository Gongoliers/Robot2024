package frc.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CustomXboxController extends CommandXboxController {

  private final double kAxisDeadband = 0.1;

  public CustomXboxController(int port) {
    super(port);
  }

  @Override
  public double getLeftX() {
    return MathUtil.applyDeadband(super.getLeftX(), kAxisDeadband);
  }

  public Trigger leftX() {
    return new Trigger(() -> getLeftX() != 0);
  }

  @Override
  public double getLeftY() {
    return MathUtil.applyDeadband(super.getLeftY(), kAxisDeadband);
  }

  public Trigger leftY() {
    return new Trigger(() -> getLeftY() != 0);
  }

  @Override
  public double getRightX() {
    return MathUtil.applyDeadband(super.getRightX(), kAxisDeadband);
  }

  public Trigger rightX() {
    return new Trigger(() -> getRightX() != 0);
  }

  @Override
  public double getRightY() {
    return MathUtil.applyDeadband(super.getRightY(), kAxisDeadband);
  }

  public Trigger rightY() {
    return new Trigger(() -> getRightY() != 0);
  }
}
