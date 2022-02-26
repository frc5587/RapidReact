package frc.robot.commands;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetTurret extends CommandBase {
    private final Turret turret;

    public SetTurret(Turret turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.resetEncoders();
        System.out.println("run  " + turret.getPositionRadians());
        turret.enable();
        turret.setDistance(Math.PI / 2);
    }
}
