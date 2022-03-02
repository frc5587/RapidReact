package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimbConstants;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.frc5587.lib.subsystems.FPIDSubsystem;

import com.revrobotics.RelativeEncoder;

public class ClimbMotor extends FPIDSubsystem {
    private static CANSparkMax motor = new CANSparkMax(ClimbConstants.INNER_CLIMB_RIGHT_MOTOR,
            MotorType.kBrushless);

    private static RelativeEncoder encoder = motor.getEncoder();
    private static FPIDConstants constants = new FPIDConstants(
        ClimbConstants.GEARING,
        ClimbConstants.SOFT_LIMITS,
        0,
        ClimbConstants.ENCODER_CPR,
        0,
        false,
        ClimbConstants.PID,
        ClimbConstants.FF_CONTROLLER,
        ClimbConstants.CONSTRAINTS);

    public ClimbMotor() {
        super(constants, motor);
    }

    public void configureMotors() {
        motor.restoreFactoryDefaults();
        motor.setInverted(ClimbConstants.MOTOR_INVERTED);
        motor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        SmartDashboard.putNumber("Output", output);
        SmartDashboard.putNumber("Setpoint", setpoint.position);
        // set(output);
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

    @Override
    public double rotationsToMeasurement(double rotations) {
        return rotations;
    }
}