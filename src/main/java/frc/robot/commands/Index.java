package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.subsystems.Conveyor.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Index extends CommandBase {
    private final Intake intake;
    private final IntakePistons intakePistons;
    private final Conveyor conveyor;
    private final Kicker rightKicker, leftKicker;
    private final LinebreakSensor linebreakSensor;
    private boolean crossed = false;

    public Index(Intake intake, IntakePistons intakePistons, Conveyor conveyor, Kicker rightKicker, Kicker leftKicker, LinebreakSensor linebreakSensor) {
        this.intake = intake;
        this.intakePistons = intakePistons;
        this.conveyor = conveyor;
        this.rightKicker = rightKicker;
        this.leftKicker = leftKicker;
        this.linebreakSensor = linebreakSensor;

        addRequirements(intake, intakePistons, conveyor, rightKicker, leftKicker, linebreakSensor);
    }

    /*
    Extend the intake before doing anything
    */
    @Override
    public void initialize() {
        // Extend the intake
        intakePistons.extend();

        // Enable Kicker PositionPID
        rightKicker.enable();
        leftKicker.enable();
        conveyor.setControlMode(ControlMode.VELOCITY);
        if(!linebreakSensor.isCrossed()) {
            conveyor.setVelocity(3);
        }
        // Set the intake velocity
        intake.setVelocity(3);

        crossed = false;
    }

    @Override
    public void execute() {
        // Check if there is a ball already in the kicker. If there isn't run the kicker until there is.
        if(linebreakSensor.isCrossed()) {
            conveyor.setVelocity(0);
            if(crossed == false) {
                rightKicker.setGoal(rightKicker.getPosition());
                leftKicker.setGoal(leftKicker.getPosition());
            }
            crossed = true;
            // rightKicker.disable();
            // leftKicker.disable();
        } else {
            rightKicker.setGoal(rightKicker.getPosition() + 0.1);
            leftKicker.setGoal(leftKicker.getPosition() + 0.1);
        }

        // TODO Implement hasBall method into conveyor
        /*
        Check if the conveyor has a ball. If it does, move the ball 1 meter into the kicker. 
            Remember that the kicker will still be running if there isn't a ball already. If there isn't, the kicker will move the ball up until its at the linebreak point.
        If the conveyor doesn't have a ball, simply run the conveyor at 0.5 velocity until it does.
        */
        // if(conveyor.hasBall()) {
        //     conveyor.setVelocity(0);
        //     conveyor.setControlMode(ControlMode.POSITION);
        //     conveyor.moveDistance(conveyor.getPosition() + 1);
        // } else {
        //     conveyor.setControlMode(ControlMode.VELOCITY);
        //     conveyor.setVelocity(0.5);
        // }
    }

    /*
    When the command ends, retract the intake, stop the intake motors, and turn off the conveyor.
    We keep the kicker on so that the PID maintains its position.
    */
    @Override
    public void end(boolean interrupted) {
        intakePistons.retract();
        intake.stop();

        conveyor.setControlMode(ControlMode.POSITION);
        conveyor.moveMore(0.2);
    }
}
