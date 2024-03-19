package frc.robot.superstructure;

import frc.robot.superstructure.SuperstructureConstants.PivotAngleConstants;
import frc.robot.superstructure.SuperstructureConstants.ShoulderAngleConstants;
import frc.robot.superstructure.SuperstructureConstants.WristAngleConstants;
import java.util.LinkedList;
import java.util.Queue;

public class SuperstructureGoals {

  private final Queue<SuperstructureState> goals;

  public static Queue<SuperstructureState> merge(
      Queue<SuperstructureState> first, Queue<SuperstructureState> second) {
    Queue<SuperstructureState> mergedGoals = new LinkedList<SuperstructureState>();

    while (!first.isEmpty()) {
      mergedGoals.add(first.poll());
    }

    while (!second.isEmpty()) {
      mergedGoals.add(second.poll());
    }

    return mergedGoals;
  }

  public static Queue<SuperstructureState> generate(
      SuperstructureState start, SuperstructureState end) {
    Queue<SuperstructureState> goals = new LinkedList<SuperstructureState>();

    boolean shoulderMove = !start.atShoulderAngleGoal(end);

    // Shoulder move: HOME -> STOW; ANY -> AMP
    // Move wrist to STOW first, then move shoulder
    if (shoulderMove) {
      goals.add(start.withWristAngle(WristAngleConstants.STOW));
      goals.add(end.withWristAngle(WristAngleConstants.STOW));
      goals.add(end);
      return goals;
    }

    boolean shoulderStowed = start.atShoulderAngle(ShoulderAngleConstants.STOW);
    boolean wristMove = !start.atWristAngleGoal(end);
    boolean pivotMove = !start.atPivotAngleGoal(end);
    boolean pivotUp = start.pivotAngleRotations().position > PivotAngleConstants.DOWN.getRotations();

    // Stow position movement & wrist-intake collision avoidance
    if (shoulderStowed && wristMove && pivotMove) {
      if (pivotUp) {
        // Pivot is up and needs to go down; move it first
        goals.add(start.withPivotAngleOf(end));
      } else {
        // Pivot is down and needs to go up; move wrist out of way first
        goals.add(start.withWristAngleOf(end));
      }
    }

    goals.add(end);

    return goals;
  }

  public SuperstructureGoals(Queue<SuperstructureState> goals) {
    this.goals = goals;
  }

  public SuperstructureGoals(SuperstructureState start, SuperstructureState end) {
    this(generate(start, end));
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
