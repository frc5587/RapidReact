package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class KickerOnly extends CommandBase {
    private final Kicker rightKicker;
    private final Kicker leftKicker;
    private final ShooterSensor shooterSensor;


    public KickerOnly(Kicker rightKicker, Kicker leftKicker, ShooterSensor shooterSensor) {
        this.rightKicker = rightKicker;
        this.leftKicker = leftKicker;
        this.shooterSensor = shooterSensor;
        addRequirements(rightKicker, leftKicker, shooterSensor);
    }

    @Override
    public void initialize() {
        rightKicker.enable();
        leftKicker.enable();

        // rightKicker.moveDistance(rightKicker.getPosition() + Units.inchesToMeters(8));
        // leftKicker.moveDistance(leftKicker.getPosition() + Units.inchesToMeters(8));
    }

    @Override
    public void execute() {
        if(shooterSensor.isCrossed()) {
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