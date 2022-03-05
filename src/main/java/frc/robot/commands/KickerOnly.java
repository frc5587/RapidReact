package frc.robot.commands;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class KickerOnly extends CommandBase {
    private final Kicker rightKicker, leftKicker;
    private final LinebreakSensor linebreakSensor;

    public KickerOnly(Kicker rightKicker, Kicker leftKicker, LinebreakSensor linebreakSensor) {
        this.rightKicker = rightKicker;
        this.leftKicker = leftKicker;
        this.linebreakSensor = linebreakSensor;

        addRequirements(rightKicker, leftKicker, linebreakSensor);
    }

    @Override
    public void initialize() {
        rightKicker.enable();
        leftKicker.enable();
    }

    @Override
    public void execute() {
        if(linebreakSensor.isCrossed()) {
            rightKicker.setGoal(rightKicker.getPosition());
            leftKicker.setGoal(leftKicker.getPosition());
            rightKicker.disable();
            leftKicker.disable();
        } else {
            rightKicker.setGoal(rightKicker.getPosition() + 1);
            leftKicker.setGoal(leftKicker.getPosition() + 1);
        }
    }
}