package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePistons extends SubsystemBase {
    public DoubleSolenoid piston = new DoubleSolenoid(PneumaticsModuleType.REVPH,0,3);
    
    public IntakePistons() {}

    public void extend() {
        piston.set(DoubleSolenoid.Value.kForward);
    }

    public void retract() {
        piston.set(DoubleSolenoid.Value.kReverse);
    }
}
