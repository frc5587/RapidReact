package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Turret;

public class SetTurret extends InstantCommand {
    private final double position;
    private final Turret turret;

    /** 
     * sets the turret to a given position
     * @param position the position in radians
     */
    public SetTurret(Turret turret, double position) {
        this.turret = turret;
        this.position = position;
    }

    @Override
    public void execute() {
        turret.setPosition(position);
    }
}
