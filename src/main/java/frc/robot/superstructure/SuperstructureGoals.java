package frc.robot.superstructure;

import java.util.LinkedList;
import java.util.Queue;

public class SuperstructureGoals {

  private final Queue<SuperstructureState> goals;

  public SuperstructureGoals(SuperstructureState start, SuperstructureState end) {
    goals = new LinkedList<SuperstructureState>();

    if (start.pivotIn() && end.pivotOut()) {
      goals.add(start.withPivotAngleOf(end));
    }

    goals.add(end);
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
