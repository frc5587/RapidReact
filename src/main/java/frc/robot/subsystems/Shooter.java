package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends ProfiledPIDSubsystem {
    private final TalonFX leaderMotor = new TalonFX(Constants.ShooterConstants.SHOOTER_LEADER_MOTOR);
    private final TalonFX followerMotor = new TalonFX(Constants.ShooterConstants.SHOOTER_FOLLOWER_MOTOR);

    public Shooter() {
        super(new ProfiledPIDController(
            Constants.ShooterConstants.PID.kP,
            Constants.ShooterConstants.PID.kI,
            Constants.ShooterConstants.PID.kD,
            Constants.ShooterConstants.CONSTRAINTS
        ));

        leaderMotor.setInverted(true);
        followerMotor.setInverted(true);
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


    public void flyWheelForwards() {
        leaderMotor.set(ControlMode.PercentOutput, Constants.ShooterConstants.FORWARDS_VELOCITY);
        followerMotor.set(ControlMode.PercentOutput, Constants.ShooterConstants.FORWARDS_VELOCITY);
    }

    public void flyWheelBackwards() {
        leaderMotor.set(ControlMode.PercentOutput, Constants.ShooterConstants.BACKWARDS_VELOCITY);
        followerMotor.set(ControlMode.PercentOutput, Constants.ShooterConstants.BACKWARDS_VELOCITY);
    }

    public void stop() {
        leaderMotor.set(ControlMode.PercentOutput, 0);
        followerMotor.set(ControlMode.PercentOutput, 0);
    }

    public void resetEncoders() {
        leaderMotor.setSelectedSensorPosition(0);
        followerMotor.setSelectedSensorPosition(0);
    }

    protected double getPositionDegrees() {
        return (leaderMotor.getSelectedSensorPosition() / followerMotor.getSelectedSensorPosition() / Constants.ShooterConstants.GEARING / Constants.ShooterConstants.ENCODER_CPR);
    }

    protected double getPositionRadians() {
        return Math.toRadians(getPositionDegrees());
    }

    @Override
    protected double getMeasurement() {
        return getPositionRadians();
    }

    // TODO Output argument should actually be used for PID
    @Override
    protected void useOutput(double output, State setpoint) {
        flyWheelForwards();
    }
}
