package frc.robot.superstructure;

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
