package frc.robot.subsystems;

import org.frc5587.lib.subsystems.SimpleMotorBase;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
* A SUPER simple intake using one SparkMax based on {@link SimpleMotorBase}.
*/
public class Intake extends ProfiledPIDSubsystem {
    private static CANSparkMax motor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR, MotorType.kBrushless);
    private static RelativeEncoder encoder = motor.getEncoder();
    public DoubleSolenoid piston = new DoubleSolenoid(PneumaticsModuleType.REVPH,0,3);

    public Intake() {
        super(new ProfiledPIDController(
            IntakeConstants.PID.kP,
            IntakeConstants.PID.kI,
            IntakeConstants.PID.kD,
            IntakeConstants.CONSTRAINTS
        ));
        configureMotors();
    }

    public void extend() {
        piston.set(DoubleSolenoid.Value.kForward);
    }

    public void retract() {
        piston.set(DoubleSolenoid.Value.kReverse);
    }

    public void setVelocity(double velocity) {
        setGoal(new TrapezoidProfile.State(0, velocity));
    }

    public void configureMotors() {
        resetEncoders();

        motor.restoreFactoryDefaults();
        motor.setInverted(IntakeConstants.INVERTED);
        motor.setIdleMode(IdleMode.kBrake);
    }

    public void moveWithThrottle(double throttle) {
        motor.set(throttle);
    }

    public void stop() {
        motor.set(0);
    }
    
    public void resetEncoders() {
        encoder.setPosition(0);
    }

    protected double getPositionRotations() {
        return (encoder.getPosition() / IntakeConstants.GEARING);
    }

    protected double getPositionRadians() {
        return getPositionRotations() * 2 * Math.PI;
    }

    @Override
    protected double getMeasurement() {
        return encoder.getVelocity() / 60 * 2 * Math.PI * Units.inchesToMeters(1) / IntakeConstants.GEARING;
    }

    // @Override
    // public void periodic() {
    //     // we should only count one rotation of the roller.
    //     if(getPositionDegrees() >= 360) {
    //         resetEncoders();
    //     }
    // }

    double lastOutput = 0;
    @Override
    protected void useOutput(double output, State setpoint) {
        lastOutput = output;
        // System.out.println(setpoint.velocity);
        // moveWithThrottle(output);        
        motor.setVoltage(IntakeConstants.INTAKE_FF.calculate(setpoint.velocity) - output);
    }
}
