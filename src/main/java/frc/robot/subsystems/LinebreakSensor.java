package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DigitalInput;

public class LinebreakSensor extends SubsystemBase {
    private final DigitalInput linebreak = new DigitalInput(9);

    public boolean isCrossed() {
        return !linebreak.get();
    }
}
