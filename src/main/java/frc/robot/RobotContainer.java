// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

import org.frc5587.lib.control.DeadbandJoystick;
import org.frc5587.lib.control.DeadbandXboxController;

import edu.wpi.first.cameraserver.CameraServer;
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
    private final DeadbandJoystick joystick = new DeadbandJoystick(0, 1.5, 0.02);
    private final DeadbandXboxController xb = new DeadbandXboxController(1);

    /* Subsystems */
    private final Drivetrain drivetrain = new Drivetrain();
    private final ClimbController climbController = new ClimbController();
    private final Intake intake = new Intake();
    private final IntakePistons intakePistons = new IntakePistons();
    private final Conveyor conveyor = new Conveyor();
    private final Kicker rightKicker = Kicker.createRightKicker();
    private final Kicker leftKicker = Kicker.createLeftKicker();
    protected final Turret turret = new Turret();
    private final Limelight limelight = new Limelight(drivetrain, turret);
    private final Shooter shooter = new Shooter(limelight);
    private final LinebreakSensor linebreakSensor = new LinebreakSensor();

    /* Commands */
    private final CurveDrive curveDrive = new CurveDrive(drivetrain, joystick::getY, () -> -joystick.getX(),
            joystick::getTrigger);
    private final ArcadeDrive arcadeDrive = new ArcadeDrive(drivetrain, joystick::getY, () -> -joystick.getX());
    private final ClimbThrottle climbThrottle = new ClimbThrottle(climbController, turret, intakePistons,
            xb::getRightY, xb::getLeftY, xb::getXButton);
    private final Index index = new Index(intake, intakePistons, conveyor, linebreakSensor, drivetrain);
    private final BottomBallOut bottomBallOut = new BottomBallOut(intake, intakePistons, conveyor);
    private final TopBallOut topBallOut = new TopBallOut(rightKicker, leftKicker, shooter);
    private final ThrottleTurret throttleTurret = new ThrottleTurret(turret, limelight, xb::getLeftX);
    private final SpinUpShooter spinUpShooter = new SpinUpShooter(shooter, drivetrain, limelight);
    private final FireWhenReady fireWhenReady = new FireWhenReady(conveyor, leftKicker, rightKicker, shooter, limelight, turret);
    private final LockTurret lockTurret = new LockTurret(turret, limelight, drivetrain, shooter);
    private final MakeClimbCoast makeClimbCoast = new MakeClimbCoast(climbController);

    private final GotoHubSpot gotoHubSpot = new GotoHubSpot(drivetrain, limelight);

    /* Misc */
    private final AutoPaths autoPaths = new AutoPaths(intake, intakePistons, conveyor, rightKicker,
            leftKicker, linebreakSensor, drivetrain, limelight, turret, shooter, climbThrottle, false);
    private final PowerDistribution pdh = new PowerDistribution();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        /* Clear sticky fault LEDs on PDH upon deploy */
        pdh.clearStickyFaults();
        /* Start USB camera capture */
        CameraServer.startAutomaticCapture();
        /* Set default commands */
        drivetrain.setDefaultCommand(curveDrive);
        // drivetrain.setDefaultCommand(arcadeDrive);
        turret.setDefaultCommand(lockTurret);
        /* Configure the button bindings */
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons are created
     * within {@link DeadbandXboxController}.
     */
    private void configureButtonBindings() {
        Trigger limelightTrigger = new Trigger(limelight::hasTarget);
        Trigger thumbTrigger = new Trigger(() -> joystick.getRawButton(2));
        
        /** when the Joystick thumb trigger is pressed, auto-drive to the ideal shooting spot */
        thumbTrigger.whileActiveOnce(gotoHubSpot);

        /*
         * INDEX
         */

        /** while the the B button is held, index */
        xb.bButton.and(xb.leftTrigger.negate()).and(xb.rightTrigger.negate()).whileActiveOnce(index);

        /*
         * EJECT
         */

        /**
         * while the B button is held with the left trigger, eject the ball through the
         * intake
         */
        xb.bButton.and(xb.leftTrigger).and(xb.rightTrigger.negate()).whileActiveOnce(bottomBallOut);
        /**
         * while the Y button is held with the left trigger, eject the ball through the
         * shooter
         */
        xb.yButton.and(xb.leftTrigger).and(xb.rightTrigger.negate()).whileActiveOnce(topBallOut);

        /*
         * TURRET
         */

        
        /**
         * Allows throttle turret if left x and no target is found, otherwise, operator
         * can override with left trigger
         */
        xb.leftStickX.and(limelightTrigger.negate().or(xb.leftTrigger)).whileActiveOnce(throttleTurret);

        /*
         * SHOOTER
         */

        /** while the A button is held, spin up the shooter to the correct speed */
        xb.aButton.and(xb.rightTrigger.negate()).whileActiveOnce(spinUpShooter);
        /** when the left bumper is pressed, fire the ball into the at-speed shooter */
        xb.leftBumper.and(xb.rightTrigger.negate()).whileActiveOnce(fireWhenReady);

        /*
         * CLIMB
         */

        /** While right trigger is held, enable the throttle climb (this blocks the
         * turret as well), and is not interruptible */
        xb.rightTrigger.and(xb.leftTrigger.negate()).whileActiveOnce(climbThrottle, false);
        xb.rightTrigger.and(xb.leftTrigger).whileActiveOnce(makeClimbCoast);
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