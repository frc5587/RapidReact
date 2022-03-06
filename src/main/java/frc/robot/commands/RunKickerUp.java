package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class RunKickerUp extends CommandBase {
    private final Conveyor conveyor;
    private final Kicker rightKicker;
    private final Kicker leftKicker;
    private final LinebreakSensor shooterSensor;


    public RunKickerUp(Conveyor conveyor, Kicker rightKicker, Kicker leftKicker, LinebreakSensor shooterSensor) {
        this.conveyor = conveyor;
        this.rightKicker = rightKicker;
        this.leftKicker = leftKicker;
        this.shooterSensor = shooterSensor;
        addRequirements(conveyor, rightKicker, leftKicker, shooterSensor);
    }

    @Override
    public void initialize() {
        rightKicker.enable();
        leftKicker.enable();

        conveyor.setControlMode(Conveyor.ControlMode.POSITION);
        
        conveyor.moveDistance(conveyor.getPosition() + Units.inchesToMeters(16));
        rightKicker.moveDistance(rightKicker.getPosition() + Units.inchesToMeters(8));
        leftKicker.moveDistance(leftKicker.getPosition() + Units.inchesToMeters(8));
    }

    @Override
    public void execute() {
        if(shooterSensor.isCrossed()) {
            rightKicker.setGoal(rightKicker.getPosition());
            leftKicker.setGoal(leftKicker.getPosition());
        } else {
            rightKicker.setGoal(rightKicker.getPosition() + 1);
            leftKicker.setGoal(leftKicker.getPosition() + 1);
        }
    }
}

