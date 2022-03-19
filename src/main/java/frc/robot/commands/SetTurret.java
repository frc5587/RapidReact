package frc.robot.commands;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetTurret extends CommandBase {
    private final Turret turret;
    private final double position;

    public SetTurret(Turret turret, double position) {
        this.turret = turret;
        this.position = position;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.enable();
    }

    @Override
    public void execute() {
        turret.setPosition(position);
    }
    
    @Override
    public boolean isFinished() {
        return turret.isFinished();
    }
}
