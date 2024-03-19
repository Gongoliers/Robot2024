package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.LinkedList;
import java.util.Queue;

public class ToGoal extends Command {

  private final Superstructure superstructure;

  private final SuperstructureState goal;

  private SuperstructureGoals goals;

  public ToGoal(SuperstructureState goal) {
    this.superstructure = Superstructure.getInstance();

    this.goal = goal;

    Queue<SuperstructureState> empty = new LinkedList<SuperstructureState>();
    goals = new SuperstructureGoals(empty);

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
