package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.*;
import frc.robot.Constants.IntakeConstants;

import frc.robot.subsystems.Conveyor.ControlMode;

/** Intakes cargo and moves balls to safe positions within the robot for shooting */
public class Index extends CommandBase {
    private final Intake intake;
    private final IntakePistons intakePistons;
    private final Conveyor conveyor;
    private final Kicker rightKicker, leftKicker;
    private final LinebreakSensor linebreakSensor;
    private final Drivetrain drivetrain;
    private boolean crossed = false;

    public Index(Intake intake, IntakePistons intakePistons, Conveyor conveyor, Kicker rightKicker, Kicker leftKicker, LinebreakSensor linebreakSensor, Drivetrain drivetrain) {
        this.intake = intake;
        this.intakePistons = intakePistons;
        this.conveyor = conveyor;
        this.rightKicker = rightKicker;
        this.leftKicker = leftKicker;
        this.linebreakSensor = linebreakSensor;
        this.drivetrain = drivetrain;

        addRequirements(intake, intakePistons, conveyor, rightKicker, leftKicker);
    }

    @Override
    public void initialize() {
        intakePistons.extend();
        rightKicker.enable();
        leftKicker.enable();
        conveyor.setControlMode(ControlMode.VELOCITY);
        /* If the linebreak sensor is not crossed, move the conveyor to push balls into its view */
        if(!linebreakSensor.isCrossed()) {
            conveyor.setVelocity(3);
        }

        crossed = false;
    }

    @Override
    public void execute() {
        /* 
         * Check if there is a ball already in the kicker. 
         * If there is, move the kicker down so the ball is just below the sensor.
         * If there isn't, run the kicker until there is. 
         */
        if(linebreakSensor.isCrossed()) {
            conveyor.setVelocity(0);
            if(crossed == false) {
                // leftKicker.moveDistance(-0.01); // this number is arbitrary but it works
                // rightKicker.moveDistance(-0.01);
            }
            crossed = true;
        } else {
            // leftKicker.moveDistance(0.1); // more arbitrary numbers
            // rightKicker.moveDistance(0.1);
        }

        /** Run intake by the speed of robot with a minimum velocity */
        intake.setVelocity(IntakeConstants.MIN_VELOCITY + drivetrain.getLinearVelocity() 
                * IntakeConstants.DRIVETRAIN_VELOCITY_OFFSET);
    }

    /*
     * When the command ends, retract the intake, stop the intake motors, and turn off the conveyor.
     * We keep the kicker on so that the PID maintains its position.
     */
    @Override
    public void end(boolean interrupted) {
        intakePistons.retract();
        intake.stop();

        conveyor.setControlMode(ControlMode.POSITION);
        conveyor.moveDistance(0.1);
    }
}
