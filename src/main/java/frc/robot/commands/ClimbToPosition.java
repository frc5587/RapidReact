package frc.robot.commands;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimbToPosition extends CommandBase {
    private Climb climb1, climb2;
    private double position;

    public ClimbToPosition(Climb climb1, Climb climb2, double position) {
        this.climb1 = climb1;
        this.climb2 = climb2;
        this.position = position;


        addRequirements(climb1, climb2);
    }

    @Override
    public void initialize() {
        climb1.enable();
        climb2.enable();
    }

    @Override
    public void execute() {
        climb1.setPosition(position);
        climb2.setPosition(position);
    }
}
