package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.frc5587.lib.subsystems.DrivetrainBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends DrivetrainBase {
    private static WPI_TalonFX leftLeader = new WPI_TalonFX(10);
    private static WPI_TalonFX leftFollower = new WPI_TalonFX(15);
    private static WPI_TalonFX rightLeader = new WPI_TalonFX(20);
    private static WPI_TalonFX rightFollower = new WPI_TalonFX(25);

    private static MotorControllerGroup left = new MotorControllerGroup(leftLeader, leftFollower);
    private static MotorControllerGroup right = new MotorControllerGroup(rightLeader, rightFollower);

    public static DriveConstants driveConstants = new DriveConstants(
        DrivetrainConstants.WHEEL_DIAMETER_METERS,
        DrivetrainConstants.HISTORY_LIMIT,
        DrivetrainConstants.INVERT_GYRO,
        DrivetrainConstants.ENCODER_EPR,
        DrivetrainConstants.GEARING
    );

    public Drivetrain() {
        super(left, right, driveConstants);
    }

    @Override
    public void configureMotors() {
        leftLeader.configFactoryDefault();
        leftFollower.configFactoryDefault();
        rightLeader.configFactoryDefault();
        rightFollower.configFactoryDefault();

        leftLeader.setNeutralMode(NeutralMode.Brake);
        leftFollower.setNeutralMode(NeutralMode.Brake);
        rightLeader.setNeutralMode(NeutralMode.Brake);
        rightFollower.setNeutralMode(NeutralMode.Brake);

        leftLeader.setInverted(DrivetrainConstants.LEFT_SIDE_INVERTED);
        leftFollower.setInverted(DrivetrainConstants.LEFT_SIDE_INVERTED);
        rightLeader.setInverted(DrivetrainConstants.RIGHT_SIDE_INVERTED);
        rightFollower.setInverted(DrivetrainConstants.RIGHT_SIDE_INVERTED);

        leftLeader.configStatorCurrentLimit(DrivetrainConstants.STATOR_CURRENT_LIMIT_CONFIGURATION);
        rightLeader.configStatorCurrentLimit(DrivetrainConstants.STATOR_CURRENT_LIMIT_CONFIGURATION);
        leftFollower.configStatorCurrentLimit(DrivetrainConstants.STATOR_CURRENT_LIMIT_CONFIGURATION);
        rightFollower.configStatorCurrentLimit(DrivetrainConstants.STATOR_CURRENT_LIMIT_CONFIGURATION);
    }

    @Override
    protected double getLeftPositionTicks() {
        return rightLeader.getSelectedSensorPosition() * (DrivetrainConstants.LEFT_ENCODERS_INVERTED ? -1:1);
    }

    @Override
    protected double getRightPositionTicks() {
        return rightLeader.getSelectedSensorPosition() * (DrivetrainConstants.RIGHT_ENCODERS_INVERTED ? -1:1);
    }

    @Override
    protected double getLeftVelocityTicksPerSecond() {
        return leftLeader.getSelectedSensorVelocity() * (DrivetrainConstants.LEFT_ENCODERS_INVERTED ? -1:1);
    }

    @Override
    protected double getRightVelocityTicksPerSecond() {
        return rightLeader.getSelectedSensorVelocity() * (DrivetrainConstants.RIGHT_ENCODERS_INVERTED ? -1:1);
    }

    @Override
    protected void resetEncoders() {
        leftLeader.setSelectedSensorPosition(0);
        rightLeader.setSelectedSensorPosition(0);
    }    
}