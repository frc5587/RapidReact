package frc.robot.subsystems;

import org.frc5587.lib.subsystems.DrivetrainBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.AutoConstants;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Drivetrain extends DrivetrainBase {
    private static final WPI_TalonFX leftLeader =new WPI_TalonFX(DrivetrainConstants.LEFT_LEADER);
    private static final WPI_TalonFX leftFollower = new WPI_TalonFX(DrivetrainConstants.LEFT_FOLLOWER);
    private static final WPI_TalonFX rightLeader = new WPI_TalonFX(DrivetrainConstants.RIGHT_LEADER);
    private static final WPI_TalonFX rightFollower= new WPI_TalonFX(DrivetrainConstants.RIGHT_FOLLOWER);
    private Field2d field = new Field2d();

    private static DriveConstants driveConstants = new DriveConstants(
        DrivetrainConstants.WHEEL_DIAMETER,
        DrivetrainConstants.HISTORY_LIMIT, 
        DrivetrainConstants.INVERT_GYRO,
        DrivetrainConstants.ENCODER_EPR, 
        DrivetrainConstants.GEARING,
        AutoConstants.TRACK_WIDTH
    );

    public Drivetrain() {
        super(
            new MotorControllerGroup(leftLeader, leftFollower), 
            new MotorControllerGroup(rightLeader, rightFollower), 
            driveConstants);
        zeroOdometry();
        SmartDashboard.putData(field);
    }

    @Override
    public void configureMotors() {
        leftLeader.configFactoryDefault();
        rightLeader.configFactoryDefault();
        leftFollower.configFactoryDefault();
        rightFollower.configFactoryDefault();

        leftLeader.setNeutralMode(NeutralMode.Brake);
        rightLeader.setNeutralMode(NeutralMode.Brake);
        leftFollower.setNeutralMode(NeutralMode.Brake);
        rightFollower.setNeutralMode(NeutralMode.Brake);

        leftLeader.setInverted(DrivetrainConstants.LEFT_MOTORS_INVERTED);
        rightLeader.setInverted(DrivetrainConstants.RIGHT_MOTORS_INVERTED);
        leftFollower.setInverted(DrivetrainConstants.LEFT_MOTORS_INVERTED);
        rightFollower.setInverted(DrivetrainConstants.RIGHT_MOTORS_INVERTED);

        leftLeader.configSupplyCurrentLimit(DrivetrainConstants.SUPPLY_CURRENT_LIMIT_CONFIGURATION);
        rightLeader.configSupplyCurrentLimit(DrivetrainConstants.SUPPLY_CURRENT_LIMIT_CONFIGURATION);
        leftFollower.configSupplyCurrentLimit(DrivetrainConstants.SUPPLY_CURRENT_LIMIT_CONFIGURATION);
        rightFollower.configSupplyCurrentLimit(DrivetrainConstants.SUPPLY_CURRENT_LIMIT_CONFIGURATION);

        leftLeader.configOpenloopRamp(DrivetrainConstants.VOLTAGE_RAMP_RATE);
        rightLeader.configOpenloopRamp(DrivetrainConstants.VOLTAGE_RAMP_RATE);
        leftFollower.configOpenloopRamp(DrivetrainConstants.VOLTAGE_RAMP_RATE);
        rightFollower.configOpenloopRamp(DrivetrainConstants.VOLTAGE_RAMP_RATE);

        resetEncoders();
    }

    @Override
    protected double getLeftPositionTicks() {
        return leftLeader.getSelectedSensorPosition() * (DrivetrainConstants.LEFT_ENCODERS_INVERTED ? -1:1);
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
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        super.tankDriveVolts(-leftVolts, -rightVolts);
    }

    @Override
    public void periodic() {
        super.periodic();

        field.setRobotPose(getPose().getX(), getPose().getY(), getRotation2d());
    }
}