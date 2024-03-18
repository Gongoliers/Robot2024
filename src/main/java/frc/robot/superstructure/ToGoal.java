package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.LinkedList;
import java.util.Queue;

public class ToGoal extends Command {

  private final Superstructure superstructure;

  private final SuperstructureState goal;

  private final Queue<SuperstructureState> intermediateGoals;

  public ToGoal(SuperstructureState goal) {
    this.superstructure = Superstructure.getInstance();

    this.goal = goal;

    this.intermediateGoals = new LinkedList<SuperstructureState>();
  }

  @Override
  public void initialize() {
    intermediateGoals.clear();

    // TODO Add more intermediate goals

    intermediateGoals.add(goal);
  }

  @Override
  public void execute() {
    if (superstructure.at(intermediateGoals.element())) {
      if (intermediateGoals.size() > 1) {
        intermediateGoals.remove();
      }
    }

    superstructure.setGoal(intermediateGoals.element());
  }

  @Override
  public boolean isFinished() {
    return superstructure.at(goal);
  }
}
