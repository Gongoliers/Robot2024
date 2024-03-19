package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;

public class ToGoal extends Command {

  private final Superstructure superstructure;

  private final SuperstructureState goal;

  private SuperstructureGoals goals;

  public ToGoal(SuperstructureState goal) {
    this.superstructure = Superstructure.getInstance();

    this.goal = goal;

    goals = new SuperstructureGoals(superstructure.getState(), this.goal);

    addRequirements(this.superstructure);
  }

  @Override
  public void initialize() {
    goals = new SuperstructureGoals(superstructure.getState(), this.goal);
  }

  @Override
  public void execute() {
    SuperstructureState goal = goals.get();

    if (superstructure.at(goal)) {
      goal = goals.next();
    }

    superstructure.setGoal(goal);
  }

  @Override
  public boolean isFinished() {
    return superstructure.at(goal);
  }
}
