package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimbConstants;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.frc5587.lib.subsystems.ElevatorBase;

import com.revrobotics.RelativeEncoder;

public class ClimbMotor extends ElevatorBase {
    private static CANSparkMax motor = new CANSparkMax(ClimbConstants.INNER_CLIMB_RIGHT_MOTOR,
            MotorType.kBrushless);

    private static RelativeEncoder encoder = motor.getEncoder();
    private static FPIDConstants constants = new FPIDConstants(
        ClimbConstants.GEARING,
        ClimbConstants.SPOOL_CIRCUMFERENCE,
        ClimbConstants.SOFT_LIMITS,
        ClimbConstants.ZERO_OFFSET_TICKS,
        ClimbConstants.ENCODER_CPR,
        ClimbConstants.PID,
        ClimbConstants.FF_CONTROLLER,
        ClimbConstants.CONSTRAINTS);

    public ClimbMotor() {
        super(constants, motor);
        SmartDashboard.getBoolean("OUTPUT ON?", false);
    }

    public void configureMotors() {
        motor.restoreFactoryDefaults();
        motor.setInverted(ClimbConstants.MOTOR_INVERTED);
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