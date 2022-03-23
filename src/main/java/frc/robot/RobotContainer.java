// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import org.frc5587.lib.control.DeadbandJoystick;
import org.frc5587.lib.control.DeadbandXboxController;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final DeadbandJoystick joystick = new DeadbandJoystick(0, 1.5);
    // private final DeadbandJoystick rightJoystick = new DeadbandJoystick(2, 1.5); // TankDrive
    private final DeadbandXboxController xb = new DeadbandXboxController(1);

    private final PowerDistribution pdh = new PowerDistribution();

    /* Subsystems */
    private final Drivetrain drivetrain = new Drivetrain();
    private final ClimbController climbController = new ClimbController();
    private final Intake intake = new Intake();
    private final IntakePistons intakePistons = new IntakePistons();
    private final Conveyor conveyor = new Conveyor();
    private final Kicker rightKicker = Kicker.createRightKicker();
    private final Kicker leftKicker = Kicker.createLeftKicker();
    private final Turret turret = new Turret();
    private final Shooter shooter = new Shooter();
    private final LinebreakSensor linebreakSensor = new LinebreakSensor();
    private final Limelight limelight = new Limelight();

    /* Commands */
    private final ArcadeDrive arcadeDrive = new ArcadeDrive(drivetrain, joystick::getY,
            () -> -joystick.getXCurveDampened());
    // private final TankDrive tankDrive = new TankDrive(drivetrain, joystick::getY,
    //         rightJoystick::getY);
    private final ClimbThrottle climbThrottle = new ClimbThrottle(climbController, turret, intakePistons, xb::getRightY,
            xb::getLeftY);
    private final ToggleIntakePistons toggleIntakePistons = new ToggleIntakePistons(intakePistons);
    private final Index index = new Index(intake, intakePistons, conveyor, rightKicker, leftKicker, linebreakSensor,
            drivetrain);
    private final BottomBallOut bottomBallOut = new BottomBallOut(intake, intakePistons, conveyor);
    private final TopBallOut topBallOut = new TopBallOut(conveyor, rightKicker, leftKicker, linebreakSensor, shooter);
    private final ThrottleTurret throttleTurret = new ThrottleTurret(turret, xb::getLeftX);
    private final SpinUpShooter spinUpShooter = new SpinUpShooter(shooter, drivetrain, turret, limelight);
    private final FireWhenReady fireWhenReady = new FireWhenReady(conveyor, leftKicker, rightKicker, shooter);
    private final LockTurret lockTurret = new LockTurret(turret, limelight, climbThrottle);

    /* Auto Paths */
    private final AutoPaths autoPaths = new AutoPaths(intake, intakePistons, conveyor, rightKicker, leftKicker,
            linebreakSensor,
            drivetrain, limelight, turret, shooter, climbThrottle);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        /* Clear sticky fault LEDs on PDH upon deploy */
        pdh.clearStickyFaults();
        /* Set default commands */
        drivetrain.setDefaultCommand(arcadeDrive);
        // drivetrain.setDefaultCommand(tankDrive);
        turret.setDefaultCommand(throttleTurret);
        /* Configure the button bindings */
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons are created
     * within {@link DeadbandXboxController}.
     */
    private void configureButtonBindings() {
        Trigger limelightTrigger = new Trigger(limelight::hasTarget);

        /*
         * INDEX
         */
        xb.bButton.and(xb.leftTrigger.negate()).and(xb.rightTrigger.negate()).whileActiveOnce(index);

        xb.bButton.and(xb.leftTrigger).and(xb.rightTrigger.negate()).whileActiveOnce(bottomBallOut);
        xb.yButton.and(xb.leftTrigger).whileActiveOnce(topBallOut);

        /*
         * TURRET
         */
        limelightTrigger.whileActiveOnce(lockTurret);

        /*
         * SHOOTER
         */
        xb.aButton.whileActiveOnce(spinUpShooter);
        xb.leftBumper.whileActiveOnce(fireWhenReady);

        /*
         * CLIMB
         */
        (xb.rightStickY.or(xb.leftStickY)).and(xb.rightTrigger)
                .whileActiveOnce(climbThrottle);

        xb.bButton.and(xb.rightTrigger)
                .whenActive(toggleIntakePistons);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * 
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoPaths.getSelectedCommand();
    }
}