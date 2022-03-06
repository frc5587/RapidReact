package frc.robot.commands;

import frc.robot.Constants.*;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunConveyorUpPosition extends CommandBase {
    private final Conveyor conveyor;
    
    public RunConveyorUpPosition(Conveyor conveyor) {
        this.conveyor = conveyor;
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        conveyor.setControlMode(Conveyor.ControlMode.POSITION);
        conveyor.moveDistance(conveyor.getPosition()+ 1);
    }

    // @Override
    // public void end(boolean interrupted) {
    //     conveyor.moveDistance(0);
    //     conveyor.setControlMode(Conveyor.ControlMode.OFF);
    // }
}
