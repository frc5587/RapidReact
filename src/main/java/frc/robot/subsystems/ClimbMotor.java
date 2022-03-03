package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimbConstants;
import org.frc5587.lib.subsystems.ElevatorBase;

public class ClimbMotor extends ElevatorBase {
    private static CANSparkMax motor = new CANSparkMax(ClimbConstants.INNER_CLIMB_RIGHT_MOTOR,
        MotorType.kBrushless);

    private static RelativeEncoder encoder = motor.getEncoder();

    public ClimbMotor(FPIDConstants constants) {
        super(constants, motor);
        SmartDashboard.putBoolean("OUTPUT ON?", false);
    }

    public void configureMotors() {
        motor.restoreFactoryDefaults();
        motor.setInverted(ClimbConstants.INNER_CLIMB_LEFT_MOTOR_INVERTED);
        motor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public double getEncoderPosition() {
        return encoder.getPosition();
    }

    @Override
    public double getEncoderVelocity() {
        return encoder.getVelocity();
    }

    @Override
    public void setEncoderPosition(double position) {
        encoder.setPosition(position);
    }
}