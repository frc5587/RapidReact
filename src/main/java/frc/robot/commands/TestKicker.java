package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.*;

public class TestKicker extends CommandBase {
    private final Kicker rightKicker, leftKicker;

    public TestKicker(Kicker rightKicker, Kicker leftKicker) {
        this.rightKicker = rightKicker;
        this.leftKicker = leftKicker;

        addRequirements(rightKicker, leftKicker);
    }

    @Override
    public void initialize() {
        rightKicker.enable();
        leftKicker.enable();

        rightKicker.moveMore(1);
        leftKicker.moveMore(1);
    }

    @Override
    public void end(boolean interrupted) {
        leftKicker.stop();
        rightKicker.stop();
    }
}
