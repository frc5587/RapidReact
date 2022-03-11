package frc.robot.commands;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class OuterClimbPosition extends CommandBase {
    private Climb outerLeftClimb, outerRightClimb;
    private double distance;

    public OuterClimbPosition(Climb outerLeftClimb, Climb outerRightClimb, double distance) {
        this.outerLeftClimb = outerLeftClimb;
        this.outerRightClimb = outerRightClimb;
        this.distance = distance;

        addRequirements(outerLeftClimb, outerRightClimb);
    }

    @Override
    public void initialize() {
        outerLeftClimb.enable();
        outerRightClimb.enable();
    }

    @Override
    public void execute() {
        outerLeftClimb.setGoal(outerLeftClimb.getPosition() + distance);
        outerRightClimb.setGoal(outerRightClimb.getPosition() + distance);
    }
}
