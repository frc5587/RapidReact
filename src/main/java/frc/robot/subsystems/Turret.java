package frc.robot.subsystems;

import frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret extends ProfiledPIDSubsystem {
    private CANSparkMax turretMotor = new CANSparkMax(TurretConstants.TURRET_MOTOR, MotorType.kBrushless);
    private RelativeEncoder encoder = turretMotor.getEncoder();
    private double lastVelocity = 0;
    private State lastSetpoint = new State(0, 0);
    private double upperLimit = TurretConstants.LIMIT;
    private double lowerLimit = -TurretConstants.LIMIT;

    public Turret() {
        super(new ProfiledPIDController(
            TurretConstants.PID.kP,
            TurretConstants.PID.kI,
            TurretConstants.PID.kD,
            TurretConstants.CONSTRAINTS
        ));
        configureTurretSpark();
    }

    public void configureTurretSpark() {
        resetEncoders();
        turretMotor.restoreFactoryDefaults();
        turretMotor.setInverted(TurretConstants.TURRET_MOTOR_INVERTED);
        turretMotor.setIdleMode(IdleMode.kBrake);
    }

    // uses radians per second
    public void setThrottle(double throttle) {
        turretMotor.set(throttle);
    }

    // public void setVelocity(double velocity) {
    //     setGoal(TrapezoidProfile.State(0, velocity));
    // }

    public void setPosition(double position) {
        if(position >= upperLimit || position <= lowerLimit) {
            System.out.println(position + " is not allowed.");
        } else {
            setGoal(position);
        }
    }

    public void setVelocityAtPosition(double position, double velocity) {
        if(position >= upperLimit || position <= lowerLimit) {
            System.out.println(position + " is not allowed.");
        } else {
            setGoal(new State(position, velocity));
        }
    }

    public void stopTurret() {
        turretMotor.set(0);
    }

    public void resetEncoders() {
        encoder.setPosition(0);
    }

    public double getPositionRadians() {
        return (encoder.getPosition() * (2 * Math.PI) / TurretConstants.GEARING);
    }

    public double getVelocity() {
        return (encoder.getVelocity() / 60 * (2 * Math.PI) / TurretConstants.GEARING);
    }

    @Override
    protected double getMeasurement() {
        return getPositionRadians();
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        if(getPositionRadians() >= upperLimit || getPositionRadians() <= lowerLimit) {
            if((setpoint.velocity < 0 && getPositionRadians() <= lowerLimit) || (setpoint.velocity > 0 && setpoint.position >= upperLimit)) {
                turretMotor.setVoltage(TurretConstants.TURRET_FF.calculate(setpoint.velocity, (setpoint.velocity - lastSetpoint.velocity)/.02) + output);
                
                SmartDashboard.putNumber("voltage", TurretConstants.TURRET_FF.calculate((setpoint.velocity - lastSetpoint.velocity)/.02) + output);
                SmartDashboard.putNumber("Goal Velocity", setpoint.velocity);
                SmartDashboard.putNumber("Goal Acceleration", (setpoint.velocity - lastSetpoint.velocity)/.02);
                SmartDashboard.putNumber("Goal Position", setpoint.position);
                
                lastSetpoint = setpoint;
            } else {
                stopTurret();
            }
        } else {
            turretMotor.setVoltage(TurretConstants.TURRET_FF.calculate(setpoint.velocity, (setpoint.velocity - lastSetpoint.velocity)/.02) + output);
            lastSetpoint = setpoint;
        }

        // System.out.println("" + output + "  " + TurretConstants.TURRET_FF.calculate(setpoint.velocity, (setpoint.velocity - lastSetpoint.velocity)/.02));
    }

    public boolean isFinished() {
        return lastSetpoint.velocity == 0;
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Position", getPositionRadians());
        SmartDashboard.putNumber("Velocity", getVelocity());
        SmartDashboard.putNumber("Acceleration", ((lastVelocity - getVelocity()) / 0.02));
        
        lastVelocity = getVelocity();
    }
}