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
    private final LinebreakSensor linebreakSensor;
    private final Drivetrain drivetrain;

    public Index(Intake intake, IntakePistons intakePistons, Conveyor conveyor, LinebreakSensor linebreakSensor, Drivetrain drivetrain) {
        this.intake = intake;
        this.intakePistons = intakePistons;
        this.conveyor = conveyor;
        this.linebreakSensor = linebreakSensor;
        this.drivetrain = drivetrain;

        addRequirements(intake, intakePistons, conveyor);
    }

    @Override
    public void initialize() {
        conveyor.setIndexRunning(true);
        
        intakePistons.extend();
        conveyor.setControlMode(ControlMode.VELOCITY);

        /* If the linebreak sensor is not crossed, move the conveyor to push balls into its view */
        if(!linebreakSensor.isCrossed()) {
            conveyor.setVelocity(3);
        }
    }

    @Override
    public void execute() {
        if(linebreakSensor.isCrossed() && !conveyor.isShooterControlled()) {
            conveyor.setVelocity(0);
        }
        /** Run intake by the speed of robot with a minimum velocity */
        intake.setVelocity(((drivetrain.getLeftVelocityMetersPerSecond() + drivetrain.getRightVelocityMetersPerSecond()) / 2
                * IntakeConstants.DRIVETRAIN_VELOCITY_OFFSET) + IntakeConstants.MIN_VELOCITY );
    }

    /*
     * When the command ends, retract the intake, stop the intake motors, and turn off the conveyor.
     * We keep the kicker on so that the PID maintains its position.
     */
    @Override
    public void end(boolean interrupted) {
        intakePistons.retract();
        intake.stop();
        
        if (!conveyor.isShooterControlled()) {
            conveyor.setControlMode(ControlMode.POSITION);
            conveyor.moveDistance(0.1);
        }

        conveyor.setIndexRunning(false);
    }
}
