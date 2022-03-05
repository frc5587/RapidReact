package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSensor extends SubsystemBase {
    private final DigitalInput linebreak = new DigitalInput(9);

    public ShooterSensor() {

    }

    public boolean isCrossed() {
        return linebreak.get() == false;
    }
    
    @Override
    public void periodic() {
        System.out.println(linebreak.get() == false);
    }
}
