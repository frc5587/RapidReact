package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.*;

public class ColorSensor extends SubsystemBase {
    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    private ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
    
    @Override
    public void periodic() {
        Color detectedColor = colorSensor.getColor();
    
        double IR = colorSensor.getIR();
    
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("IR", IR);
    
        /*
        We want this because we only want to detect balls in our conveyor, not on the field
        */
        int proximity = colorSensor.getProximity();
    
        SmartDashboard.putNumber("Proximity", proximity);
    }
}
