package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.*;
import frc.robot.subsystems.Conveyor.ControlMode;

/** Ejects the top cargo out of the shooter */
public class TopBallOut extends CommandBase {
    private final Conveyor conveyor;
    private final Kicker rightKicker, leftKicker;
    private final LinebreakSensor linebreakSensor;
    private final Shooter shooter;
    private boolean linebroken = false;
    private boolean wasCrossedInBeginning = false;

    public TopBallOut(Conveyor conveyor, Kicker rightKicker, Kicker leftKicker, LinebreakSensor linebreakSensor, Shooter shooter) {
        this.conveyor = conveyor;
        this.rightKicker = rightKicker;
        this.leftKicker = leftKicker;
        this.linebreakSensor = linebreakSensor;
        this.shooter = shooter;

        addRequirements(conveyor, rightKicker, leftKicker, shooter);
    }

    /*
    Enable kicker PID
    */
    @Override
    public void initialize() {
        rightKicker.enable();
        leftKicker.enable();

        shooter.enable();
        shooter.setVelocity(10);
        /* if the linebreak sensor is not crossed, move the conveyor until the ball is in it */
        if(!linebreakSensor.isCrossed()) {
            conveyor.setControlMode(ControlMode.VELOCITY);
            conveyor.setVelocity(1);
        }
        /* if the linebreak sensor is crossed, push up the kickers  */
        else {
            rightKicker.moveDistance(-.2);
            leftKicker.moveDistance(-.2);
            wasCrossedInBeginning = true;
        }

        linebroken = false;
    }

    @Override
    public void execute() {
        /* 
         * if the linebreak sensor was crossed at initialize and the kickers aren't moving,
         * or if the linebreaksensor is no longer crossed, set the crossed variable to false
         */
        if (wasCrossedInBeginning && (rightKicker.isDone() || leftKicker.isDone())) {
            wasCrossedInBeginning = false;
        } else if (wasCrossedInBeginning && !linebreakSensor.isCrossed()) {
            wasCrossedInBeginning = false;
        }
        
        /* if the crossed variable is true, don't continue the command */
        if (wasCrossedInBeginning) {
            return;
        }

        /* While the ball is still in the robot, move it slowly to pop it out of the robot. */
        if (!linebreakSensor.isCrossed() && !linebroken) {
            rightKicker.moveDistance(1);
            leftKicker.moveDistance(1);
        } else {
            if(!linebroken) {
                conveyor.setControlMode(ControlMode.POSITION);
                conveyor.moveDistance(0.5);
                rightKicker.moveDistance(.3);
                leftKicker.moveDistance(.3);
            }
            linebroken = true;
        }
    }

    /* When the command ends, turn off the conveyor */
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