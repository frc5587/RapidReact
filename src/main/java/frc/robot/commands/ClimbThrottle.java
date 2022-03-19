package frc.robot.commands;

import frc.robot.subsystems.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimbThrottle extends CommandBase {
    private ClimbController climb;
    private DoubleSupplier hookThrottle, stickThrottle;
    public boolean isClimbing;
    private final Turret turret;
    private final IntakePistons intake;

    public ClimbThrottle(ClimbController climb, Turret turret, DoubleSupplier hookThrottle, DoubleSupplier stickThrottle, IntakePistons intake) {
        this.climb = climb;
        this.turret = turret;
        this.hookThrottle = hookThrottle;
        this.stickThrottle = stickThrottle;
        this.intake = intake;

        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.disable();
        isClimbing = true;
        turret.enable();
        turret.setPosition(0);
        intake.extend();
    }

    @Override
    public void execute() {
        climb.setHookThrottle(hookThrottle.getAsDouble());
        climb.setStickThrottle(stickThrottle.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        climb.setHookThrottle(0);
        climb.setStickThrottle(0);
        intake.retract();
    }
}
