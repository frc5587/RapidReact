package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.subsystems.Conveyor.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TopBallOut extends CommandBase {
    private final Conveyor conveyor;
    private final Kicker rightKicker, leftKicker;
    private final LinebreakSensor linebreakSensor;
    private final Shooter shooter;
    private boolean linebroken = false;

    public TopBallOut(Conveyor conveyor, Kicker rightKicker, Kicker leftKicker, LinebreakSensor linebreakSensor, Shooter shooter) {
        this.conveyor = conveyor;
        this.rightKicker = rightKicker;
        this.leftKicker = leftKicker;
        this.linebreakSensor = linebreakSensor;
        this.shooter = shooter;

        addRequirements(conveyor, rightKicker, leftKicker, linebreakSensor, shooter);
    }

    /*
    Enable kicker PID
    */
    @Override
    public void initialize() {
        rightKicker.enable();
        leftKicker.enable();

        shooter.enable();
        shooter.setVelocity(5);
        if(!linebreakSensor.isCrossed()) {
            conveyor.setControlMode(ControlMode.VELOCITY);
            conveyor.setVelocity(1);
        }

        linebroken = false;
    }

    @Override
    public void execute() {
        // While the ball is still in the robot, move it slowly to pop it out of the robot.
        if (!linebreakSensor.isCrossed() && !linebroken) {
            rightKicker.moveMore(1);
            leftKicker.moveMore(1);
        } else {
            if(!linebroken) {
                conveyor.setControlMode(ControlMode.POSITION);
                conveyor.moveMore(0.5);
                rightKicker.moveMore(.3);
                leftKicker.moveMore(.3);
            }
            linebroken = true;
        }
        // rightKicker.setGoal(rightKicker.getPosition());
        // leftKicker.setGoal(leftKicker.getPosition());
    }

    /*
    When the command ends, turn off the conveyor
    */
    @Override
    public void end(boolean interruptable) {
        conveyor.setControlMode(ControlMode.OFF);
        rightKicker.disable();
        leftKicker.disable();
        shooter.disable();
    }

    @Override
    public boolean isFinished() {
        return !linebreakSensor.isCrossed() && linebroken;
    }
} 