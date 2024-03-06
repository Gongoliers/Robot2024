package frc.robot.arm;

import edu.wpi.first.wpilibj2.command.Command;

public class MoveShoulderCommand extends Command {

  private final Arm arm;

  private final ArmState shoulderGoal;

  private ArmState before;

  public MoveShoulderCommand(ArmState goal) {
    this.arm = Arm.getInstance();

    this.shoulderGoal = goal;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    // Save the position of the arm before the shoulder starts moving
    before = arm.getMeasuredState().position();

    arm.setGoal(shoulderGoal.withWrist(before.wrist()));
  }

  @Override
  public void end(boolean interrupted) {
    ArmState after = arm.getMeasuredState().position();

    // Since only the shoulder should have moved, assume that the wrist is in the same position as
    // it was at the start of the command
    arm.setPosition(after.withWrist(before.wrist()));
  }

  @Override
  public boolean isFinished() {
    return arm.getMeasuredState().atShoulderGoal(shoulderGoal);
  }
}
