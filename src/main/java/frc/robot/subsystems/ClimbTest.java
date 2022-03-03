package frc.robot.subsystems;

import frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbTest extends SubsystemBase {
    private final CANSparkMax rightClimbMotor = new CANSparkMax(ClimbConstants.OUTER_CLIMB_RIGHT_MOTOR, MotorType.kBrushless);
    private final CANSparkMax leftClimbMotor = new CANSparkMax(ClimbConstants.INNER_CLIMB_LEFT_MOTOR, MotorType.kBrushless);

    private MotorControllerGroup climbMotors = new MotorControllerGroup(rightClimbMotor, leftClimbMotor);

    public ClimbTest() {
        configureMotors();
    }

    public void configureMotors() {
        rightClimbMotor.restoreFactoryDefaults();
        leftClimbMotor.restoreFactoryDefaults();

        rightClimbMotor.setInverted(ClimbConstants.OUTER_CLIMB_RIGHT_MOTOR_INVERTED);
        leftClimbMotor.setInverted(ClimbConstants.INNER_CLIMB_LEFT_MOTOR_INVERTED);

        rightClimbMotor.setIdleMode(IdleMode.kBrake);
        leftClimbMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setThrottle(double throttle) {
        climbMotors.set(throttle);
    }
}
