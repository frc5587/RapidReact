package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.subsystems.Conveyor.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveDown extends CommandBase {
    private final Intake intake;
    private final IntakePistons intakePistons;
    private final Conveyor conveyor;
    private final Kicker rightKicker, leftKicker;
    private final LinebreakSensor linebreakSensor;

    public MoveDown(Intake intake, IntakePistons intakePistons, Conveyor conveyor, Kicker rightKicker, Kicker leftKicker, LinebreakSensor linebreakSensor) {
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
        conveyor.setVelocity(-3);
        // Set the intake velocity
        intake.setVelocity(-3);
    }

    @Override
    public void execute() {
        // Check if there is a ball already in the kicker. If there isn't run the kicker until there is.
        rightKicker.setGoal(rightKicker.getPosition() - 1);
        leftKicker.setGoal(leftKicker.getPosition() - 1);
    }

    /*
    When the command ends, retract the intake, stop the intake motors, and turn off the conveyor.
    We keep the kicker on so that the PID maintains its position.
    */
    @Override
    public void end(boolean interrupted) {
        intakePistons.retract();
        intake.stop();

        conveyor.setVelocity(0); // ensure velocity has already ended to avoid RuntimeException
        conveyor.setControlMode(ControlMode.OFF);
    }   
}
