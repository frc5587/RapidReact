package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.subsystems.Conveyor.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TopBallOut extends CommandBase {
    private final Conveyor conveyor;
    private final Kicker rightKicker, leftKicker;
    private final LinebreakSensor linebreakSensor;
    private final Shooter shooter;

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
    }

    @Override
    public void execute() {
        // While the ball is still in the robot, move it slowly to pop it out of the robot.
        while(linebreakSensor.isCrossed()) {
            rightKicker.moveDistance(1);
            leftKicker.moveDistance(1);
            shooter.setVelocity(5);
        }
        rightKicker.setGoal(rightKicker.getPosition());
        leftKicker.setGoal(leftKicker.getPosition());
        shooter.stop();
        
        // Start moving up the new ball, and stop moving it when it reaches the linebreak sensor.
        if(linebreakSensor.isCrossed()) {
            rightKicker.setGoal(rightKicker.getPosition());
            leftKicker.setGoal(leftKicker.getPosition());
        } else {
            rightKicker.setGoal(rightKicker.getPosition() + 1);
            leftKicker.setGoal(leftKicker.getPosition() + 1);
        }

        // Move the conveyor forwards once into the kicker.
        conveyor.setControlMode(ControlMode.POSITION);
        conveyor.moveDistance(conveyor.getPosition() + 1);
    }

    /*
    When the command ends, turn off the conveyor
    */
    @Override
    public void end(boolean interruptable) {
        conveyor.setControlMode(ControlMode.OFF);
    }
} 