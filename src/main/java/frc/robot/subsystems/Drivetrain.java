package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.frc5587.lib.subsystems.DrivetrainBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends DrivetrainBase {
    private static final WPI_TalonFX leftLeader = new WPI_TalonFX(DrivetrainConstants.LEFT_LEADER);
    private static final WPI_TalonFX leftFollower = new WPI_TalonFX(DrivetrainConstants.LEFT_FOLLOWER);
    private static final WPI_TalonFX rightLeader = new WPI_TalonFX(DrivetrainConstants.RIGHT_LEADER);
    private static final WPI_TalonFX rightFollower = new WPI_TalonFX(DrivetrainConstants.RIGHT_FOLLOWER);
    private Field2d field = new Field2d();

    private static DriveConstants driveConstants = new DriveConstants(
        DrivetrainConstants.WHEEL_DIAMETER,
        DrivetrainConstants.HISTORY_LIMIT, 
        DrivetrainConstants.INVERT_GYRO,
        DrivetrainConstants.ENCODER_EPR, 
        DrivetrainConstants.GEARING
    );

    public Drivetrain() {
        // this(new WPI_TalonFX(DrivetrainConstants.LEFT_LEADER), new WPI_TalonFX(DrivetrainConstants.LEFT_FOLLOWER),
        //         new WPI_TalonFX(DrivetrainConstants.RIGHT_LEADER), new WPI_TalonFX(DrivetrainConstants.RIGHT_FOLLOWER));
        super(new MotorControllerGroup(leftLeader, leftFollower), new MotorControllerGroup(rightLeader, rightFollower), 
        driveConstants);
        zeroOdometry();
        SmartDashboard.putData(field);
    }

    // public Drivetrain(WPI_TalonFX leftLeader, WPI_TalonFX leftFollower, WPI_TalonFX rightLeader, WPI_TalonFX rightFollower) {
    //     super(new MotorControllerGroup(leftLeader, leftFollower), new MotorControllerGroup(rightLeader, rightFollower), 
    //     driveConstants);

    //     this.leftLeader = leftLeader;
    //     this.leftFollower = leftFollower;
    //     this.rightLeader = rightLeader;
    //     this.rightFollower = rightFollower;
    // }

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

        resetEncoders();
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
        return (leftLeader.getSelectedSensorVelocity() * (DrivetrainConstants.LEFT_ENCODERS_INVERTED ? -1:1) / DrivetrainConstants.VELOCITY_DENOMINATOR);
    }

    @Override
    protected double getRightVelocityTicksPerSecond() {
        return (rightLeader.getSelectedSensorVelocity() * (DrivetrainConstants.RIGHT_ENCODERS_INVERTED ? -1:1) / DrivetrainConstants.VELOCITY_DENOMINATOR);
    }

    @Override
    protected void resetEncoders() {
        leftLeader.setSelectedSensorPosition(0);
        rightLeader.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
        super.periodic();
        field.setRobotPose(getPose().getX(), getPose().getY(), getRotation2d());
        SmartDashboard.putNumber("Pose X", getPose().getX());
        SmartDashboard.putNumber("Pose Y", getPose().getY());
        SmartDashboard.putNumber("Angle", getHeading());
        SmartDashboard.putNumber("LeftEncoder", getLeftPositionTicks());
        SmartDashboard.putNumber("RightEncoder", getRightPositionTicks());
        SmartDashboard.putNumber("LeftDistanceEncoder", getLeftPositionMeters());
        SmartDashboard.putNumber("RightDistanceEncoder", getRightPositionMeters());
    }
}