package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Constants.ConveyorConstants;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class Conveyor extends ProfiledPIDSubsystem {
    private static CANSparkMax conveyorMotorMain = new CANSparkMax(ConveyorConstants.CONVEYOR_MOTOR_MAIN, MotorType.kBrushless);
    private static CANSparkMax conveyorMotor2 = new CANSparkMax(ConveyorConstants.CONVEYOR_MOTOR_2, MotorType.kBrushless);
    private static CANSparkMax conveyorMotor3 = new CANSparkMax(ConveyorConstants.CONVEYOR_MOTOR_3, MotorType.kBrushless);

    private static RelativeEncoder encoderMain = conveyorMotorMain.getEncoder();
    private static RelativeEncoder encoder2 = conveyorMotor2.getEncoder();
    private static RelativeEncoder encoder3 = conveyorMotor3.getEncoder();

    // private static MotorControllerGroup motorControllerGroup = new MotorControllerGroup(conveyorMotorMain, conveyorMotor2, conveyorMotor3);

    public Conveyor() {
        super(new ProfiledPIDController(
            Constants.ConveyorConstants.PID.kP,
            Constants.ConveyorConstants.PID.kI,
            Constants.ConveyorConstants.PID.kD,
            Constants.ConveyorConstants.CONSTRAINTS
        ));
        configureConveyorSpark();
    }



    public void configureConveyorSpark() {
        conveyorMotorMain.restoreFactoryDefaults();
        conveyorMotor2.restoreFactoryDefaults();
        conveyorMotor3.restoreFactoryDefaults();

        conveyorMotorMain.setInverted(ConveyorConstants.MOTOR_INVERTED);
        conveyorMotor2.setInverted(ConveyorConstants.MOTOR_INVERTED);
        conveyorMotor3.setInverted(ConveyorConstants.MOTOR_INVERTED);

        conveyorMotorMain.setIdleMode(IdleMode.kCoast);
        conveyorMotor2.setIdleMode(IdleMode.kCoast);
        conveyorMotor3.setIdleMode(IdleMode.kCoast);
    }

    public void setConveyor(double conveyorSpeed) {
        conveyorMotorMain.setIdleMode(IdleMode.kCoast);
        conveyorMotor2.setIdleMode(IdleMode.kCoast);
        conveyorMotor3.setIdleMode(IdleMode.kCoast);

        conveyorMotorMain.set(-conveyorSpeed);
        conveyorMotor2.set(-conveyorSpeed);
        conveyorMotor3.set(-conveyorSpeed);
    }

    public void stopConveyor() {
        conveyorMotorMain.set(0);
        conveyorMotor2.set(0);
        conveyorMotor3.set(0);

        conveyorMotorMain.setIdleMode(IdleMode.kBrake);
        conveyorMotor2.setIdleMode(IdleMode.kBrake);
        conveyorMotor3.setIdleMode(IdleMode.kBrake);
    }

    public void resetEncoders() {
        encoderMain.setPosition(0);
        encoder2.setPosition(0);
        encoder3.setPosition(0);
    }

    protected double getPositionDegrees() {
        return (encoderMain.getPosition() / encoder2.getPosition() / encoder3.getPosition() / Constants.ConveyorConstants.GEARING / Constants.ConveyorConstants.ENCODER_CPR);
    }

    protected double getPositionRadians() {
        return Math.toRadians(getPositionDegrees());
    }

    @Override
    protected double getMeasurement() {
        return getPositionRadians();
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        System.out.println("Test later");
    }
}