package frc.robot.commands;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Conveyor.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

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

        crossed = false;
    }

    @Override
    public void execute() {
        // Check if there is a ball already in the kicker. If there isn't run the kicker until there is.
        if(linebreakSensor.isCrossed()) {
            conveyor.setVelocity(0);
            if(crossed == false) {
                rightKicker.setGoal(rightKicker.getPosition() - 0.1);
                leftKicker.setGoal(leftKicker.getPosition() - 0.1);
            }
            crossed = true;
            // rightKicker.disable();
            // leftKicker.disable();
        } else {
            rightKicker.setGoal(rightKicker.getPosition() + 0.1);
            leftKicker.setGoal(leftKicker.getPosition() + 0.1);
        }

        // Run intake at 2x speed of robot with a min velocity
        intake.setVelocity(IntakeConstants.MIN_VELOCITY + (Math.abs(drivetrain.getLeftVelocityMetersPerSecond()) + Math.abs(drivetrain.getRightVelocityMetersPerSecond())));
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
        conveyor.moveMore(0.4);
    }
}
