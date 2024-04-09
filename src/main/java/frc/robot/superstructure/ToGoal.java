package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;

public class ToGoal extends Command {

    private final Superstructure superstructure;

    private final SuperstructureState goal;

    private SuperstructureGoals goals;
   
    public ToGoal(SuperstructureState goal) {
        superstructure = Superstructure.getInstance();

        this.goal = goal;

        goals = SuperstructureGoals.generate(superstructure.getState(), goal);

        addRequirements(superstructure);
    }

    @Override
    public void initialize() {
        goals = SuperstructureGoals.generate(superstructure.getState(), goal);
    }

    @Override
    public void execute() {
        if (superstructure.at(goals.get())) {
            superstructure.setGoal(goals.next());
        } else {
            superstructure.setGoal(goals.get());
        }
    }

}
