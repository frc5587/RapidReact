package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSensor extends SubsystemBase {
    private final DigitalInput linebreak = new DigitalInput(9);

    public ShooterSensor() {

    }

    @Override
    public void periodic() {
        System.out.println(linebreak.get());
    }
}
