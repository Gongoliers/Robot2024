package frc.robot.arm;

import edu.wpi.first.wpilibj2.command.Command;

public class MoveWristCommand extends Command {

  private final Arm arm;

  private final ArmState wristGoal;

  private ArmState before;

  public MoveWristCommand(ArmState goal) {
    this.arm = Arm.getInstance();

    this.wristGoal = goal;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    // Save the position of the arm before the wrist starts moving
    before = arm.getMeasuredState().position();

    arm.setGoal(wristGoal.withShoulder(before.shoulder()));
    arm.setSetpoint(before);
  }

  @Override
  public void execute() {
    ArmState nextSetpoint = arm.getSetpoint().nextWristSetpoint(wristGoal);

    arm.setSetpoint(nextSetpoint);
    arm.applySetpoint(nextSetpoint);
  }

  @Override
  public void end(boolean interrupted) {
    ArmState after = arm.getMeasuredState().position();

    // Since only the wrist *should* have moved, assume that the shoulder is in the same position as
    // it was at the start of the command
    arm.setPosition(after.withShoulder(before.shoulder()));
  }

  @Override
  public boolean isFinished() {
    return arm.getMeasuredState().atWristGoal(wristGoal);
  }
}
