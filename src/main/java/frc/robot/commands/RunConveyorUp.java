package frc.robot.commands;

import frc.robot.Constants.*;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunConveyorUp extends CommandBase {
    private final Conveyor conveyor;
    
    public RunConveyorUp(Conveyor conveyor) {
        this.conveyor = conveyor;
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        conveyor.setVelocity(ConveyorConstants.VELOCITY_FORWARD);
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.setVelocity(0);
        
    }
}
