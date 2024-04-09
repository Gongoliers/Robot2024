package frc.robot.superstructure;

import java.util.LinkedList;
import java.util.Queue;

public class SuperstructureGoals {

  private final Queue<SuperstructureState> goals;

  public static SuperstructureGoals generate(
      SuperstructureState start, SuperstructureState end) {
    Queue<SuperstructureState> goals = new LinkedList<SuperstructureState>();

    goals.add(end);

    return new SuperstructureGoals(goals);
  }

  public SuperstructureGoals(Queue<SuperstructureState> goals) {
    this.goals = goals;
  }

  public SuperstructureState get() {
    return goals.element();
  }

  public SuperstructureState next() {
    boolean hasNext = goals.size() > 1;

    if (hasNext) {
      goals.remove();
    }

    return get();
  }
}
