package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.*;

public class ColorSensor extends SubsystemBase {
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private Color detectedColor;
    private double IR;

    private ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
    
    // TODO Check detected colors & what the color values should be (color enums or RGB)
    public boolean hasWrongBall() {
        if((DriverStation.getAlliance() == Alliance.Red && detectedColor == Color.kBlue) || (DriverStation.getAlliance() == Alliance.Blue && detectedColor == Color.kRed)) {
            return true;
        } else {
            return false;
        }
        
    }

    @Override
    public void periodic() {
        detectedColor = colorSensor.getColor();
        IR = colorSensor.getIR();
    
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
