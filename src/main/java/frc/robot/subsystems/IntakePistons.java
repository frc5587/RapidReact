package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.*;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DoubleSolenoid.*;

/**
Extend the intake to allow for ball intake & outtake.
 */
public class IntakePistons extends SubsystemBase {
    private final DoubleSolenoid piston = new DoubleSolenoid(PneumaticsModuleType.REVPH, IntakePistonsConstants.FORWARD_CHANNEL, IntakePistonsConstants.REVERSE_CHANNEL);
    
    public IntakePistons() {}

    public void extend() {
        piston.set(DoubleSolenoid.Value.kForward);
    }

    public void retract() {
        piston.set(DoubleSolenoid.Value.kReverse);
    }

    public boolean isExtended() {
        if(piston.get() == Value.kForward) {
            return true;
        } else {
            return false;
        }
    }
}
