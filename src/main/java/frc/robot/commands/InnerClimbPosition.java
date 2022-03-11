package frc.robot.commands;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class InnerClimbPosition extends CommandBase {
    private Climb innerLeftClimb, innerRightClimb;
    private double distance;

    public InnerClimbPosition(Climb innerLeftClimb, Climb innerRightClimb, double distance) {
        this.innerLeftClimb = innerLeftClimb;
        this.innerRightClimb = innerRightClimb;
        this.distance = distance;


        addRequirements(innerLeftClimb, innerRightClimb);
    }

    @Override
    public void initialize() {
        innerLeftClimb.enable();
        innerRightClimb.enable();
    }

    @Override
    public void execute() {
        innerLeftClimb.setGoal(innerLeftClimb.getPosition() + distance);
        innerRightClimb.setGoal(innerRightClimb.getPosition() + distance);
    }
}
