package frc.robot.superstructure;

import frc.robot.superstructure.SuperstructureConstants.WristAngleConstants;
import java.util.LinkedList;
import java.util.Queue;

public class SuperstructureGoals {

  private final Queue<SuperstructureState> goals;

  public SuperstructureGoals(SuperstructureState start, SuperstructureState end) {
    goals = new LinkedList<SuperstructureState>();

    if (!start.atShoulderAngleGoal(end)) {
      goals.add(start.withWristAngle(WristAngleConstants.STOW));
      goals.add(
          start
              .withShoulderAngleOf(end)
              .withWristAngle(WristAngleConstants.STOW)
              .withPivotAngleOf(end));
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
