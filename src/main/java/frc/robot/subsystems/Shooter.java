package frc.robot.subsystems;

import frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.*;

// import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final WPI_TalonFX leaderMotor = new WPI_TalonFX(ShooterConstants.SHOOTER_LEADER_MOTOR);
    private final WPI_TalonFX followerMotor = new WPI_TalonFX(ShooterConstants.SHOOTER_FOLLOWER_MOTOR);

    private double velocitySetpoint = 0;

    public Shooter() {
        configureShooterFalcon();
    }

    public void configureShooterFalcon() {
        leaderMotor.configFactoryDefault();
        followerMotor.configFactoryDefault();

        leaderMotor.setInverted(ShooterConstants.SHOOTER_LEADER_INVERTED);
        followerMotor.setInverted(ShooterConstants.SHOOTER_FOLLOWER_INVERTED);

        leaderMotor.setNeutralMode(NeutralMode.Brake);
        followerMotor.setNeutralMode(NeutralMode.Brake);

        resetEncoders();
    }

    public void setVelocity(double velocity) {
        velocitySetpoint = velocity;
    }

    public void setThrottle(double throttle) {
        leaderMotor.set(ControlMode.PercentOutput, throttle);
        followerMotor.set(ControlMode.PercentOutput, throttle);
    }

    public void stop() {
        leaderMotor.set(ControlMode.PercentOutput, 0);
        followerMotor.set(ControlMode.PercentOutput, 0);
        velocitySetpoint = 0;
    }

    public void resetEncoders() {
        leaderMotor.setSelectedSensorPosition(0);
        followerMotor.setSelectedSensorPosition(0);
    }

    public double getVelocity() {
        return (leaderMotor.getSelectedSensorPosition() / 60 * (2 * Math.PI) * (ShooterConstants.WHEEL_RADIUS / ShooterConstants.GEARING));
    }

    @Override
    public void periodic() {
        super.periodic();

        leaderMotor.setVoltage(ShooterConstants.SHOOTER_FF.calculate(velocitySetpoint) + ShooterConstants.PID.calculate(velocitySetpoint - getVelocity()));
        followerMotor.setVoltage(ShooterConstants.SHOOTER_FF.calculate(velocitySetpoint) + ShooterConstants.PID.calculate(velocitySetpoint - getVelocity()));
    }
}
