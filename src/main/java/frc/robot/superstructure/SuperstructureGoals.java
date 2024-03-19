package frc.robot.superstructure;

import frc.robot.superstructure.SuperstructureConstants.PivotAngleConstants;
import frc.robot.superstructure.SuperstructureConstants.ShoulderAngleConstants;
import frc.robot.superstructure.SuperstructureConstants.WristAngleConstants;
import java.util.LinkedList;
import java.util.Queue;

public class SuperstructureGoals {

  private final Queue<SuperstructureState> goals;

  public static Queue<SuperstructureState> generate(
      SuperstructureState start, SuperstructureState end) {
    Queue<SuperstructureState> goals = new LinkedList<SuperstructureState>();

    // TOOD Bug where the shoulder "bounces" while the intake is pivoting down

    boolean shoulderMove = !start.atShoulderAngleGoal(end);

    // Shoulder move: HOME -> STOW; ANY -> AMP
    // Move wrist to STOW first, then move shoulder
    if (shoulderMove) {
      System.out.println("*** GENERATING SHOULDER MOVE ***");
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
    if (shoulderStowed && !shoulderMove && wristMove && pivotMove) {
      if (pivotUp) {
        // Pivot is up and needs to go down; move it first
        System.out.println("*** GENERATING PIVOT DOWN ***");
        goals.add(start.withPivotAngleOf(end));
        goals.add(end);
        return goals;
      } else {
        System.out.println("*** GENERATING WRIST UP ***");
        // Pivot is down and needs to go up; move wrist out of way first
        goals.add(start.withWristAngleOf(end));
        goals.add(end);
        return goals;
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
