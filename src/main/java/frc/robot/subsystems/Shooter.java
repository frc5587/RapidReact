package frc.robot.subsystems;

import frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.*;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final WPI_TalonFX leaderMotor = new WPI_TalonFX(ShooterConstants.SHOOTER_LEADER_MOTOR);
    private final WPI_TalonFX followerMotor = new WPI_TalonFX(ShooterConstants.SHOOTER_FOLLOWER_MOTOR);

    private final MotorControllerGroup shooterMotors = new MotorControllerGroup(leaderMotor, followerMotor);

    private double setpoint = 0;

    public Shooter() {
        configureShooterFalcon();

        SmartDashboard.setDefaultNumber("Velocity", 0);
    }

    public void configureShooterFalcon() {
        resetEncoders();
        leaderMotor.configFactoryDefault();
        followerMotor.configFactoryDefault();

        leaderMotor.setInverted(ShooterConstants.SHOOTER_LEADER_INVERTED);
        followerMotor.setInverted(ShooterConstants.SHOOTER_FOLLOWER_INVERTED);

        leaderMotor.setNeutralMode(NeutralMode.Coast);
        followerMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void setVelocity(double velocity) {
        setpoint = velocity;
    }

    // public void setThrottle(double throttle) {
    //     shooterMotors.set(throttle);
    // }

    public void stop() {
        shooterMotors.set(0);
        setpoint = 0;
    }

    public void resetEncoders() {
        leaderMotor.setSelectedSensorPosition(0);
        followerMotor.setSelectedSensorPosition(0);
    }

    public double getVelocity() {
        return (leaderMotor.getSelectedSensorVelocity() / (ShooterConstants.ENCODER_EPR * ShooterConstants.VELOCITY_DENOMINATOR) * (2 * Math.PI) * (ShooterConstants.WHEEL_RADIUS / ShooterConstants.GEARING));
    }

    // public double getVelocity() {
    //     return ((leaderMotor.getSelectedSensorVelocity() / 60) * (2 * Math.PI) * (ShooterConstants.WHEEL_RADIUS / ShooterConstants.GEARING));
    // }

    public double getSmartDashboard() {
        // System.out.println(SmartDashboard.getEntry("Velocity").exists() + "   " + SmartDashboard.getEntry("Velocity").getDouble(-1) + "  " + SmartDashboard.getNumber("Velocity", 0));
        return SmartDashboard.getNumber("Velocity", 0);
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Setpoint", setpoint);
        SmartDashboard.putNumber("PID Output", ShooterConstants.PID.calculate(setpoint - getVelocity()));
        SmartDashboard.putNumber("Feed Forward", ShooterConstants.SHOOTER_FF.calculate(setpoint));
        SmartDashboard.putNumber("Voltage", ShooterConstants.SHOOTER_FF.calculate(setpoint) - ShooterConstants.PID.calculate(setpoint - getVelocity()));
        SmartDashboard.putNumber("Real Velocity", getVelocity());

        shooterMotors.setVoltage(ShooterConstants.SHOOTER_FF.calculate(setpoint) - ShooterConstants.PID.calculate(setpoint - getVelocity()));
        // setVelocity(-(getSmartDashboard()));
        if(getVelocity() != 0)
            System.out.println(getVelocity() + "  " + getSmartDashboard() + "  " + ShooterConstants.PID.calculate(setpoint - getVelocity()));
    }
}
