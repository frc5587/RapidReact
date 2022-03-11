// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

import org.frc5587.lib.auto.*;
import org.frc5587.lib.control.*;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;

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
    // Controllers
    private final DeadbandJoystick joystick = new DeadbandJoystick(0, 1.5);
    // private final DeadbandJoystick rightJoystick = new DeadbandJoystick(2, 1.5);
    // // for TankDrive
    private final DeadbandXboxController xb = new DeadbandXboxController(1, 1.5);

    // Subsystems
    private final Drivetrain drivetrain = new Drivetrain();
    private final IntakePistons intakePistons = new IntakePistons();
    private final Intake intake = new Intake();
    private final Conveyor conveyor = new Conveyor();
    private final Kicker rightKicker = Kicker.createRightKicker();
    private final Kicker leftKicker = Kicker.createLeftKicker();
    private final LinebreakSensor linebreakSensor = new LinebreakSensor();
    private final Limelight limelight = new Limelight();
    private final Turret turret = new Turret();
    private final Shooter shooter = new Shooter();

    // Commands
    private final ArcadeDrive arcadeDrive = new ArcadeDrive(drivetrain, joystick::getY,
            () -> -joystick.getXCurveDampened());
    // private final TankDrive tankDrive = new TankDrive(drivetrain, joystick::getY,
    // rightJoystick::getY);
    private final Index index = new Index(intake, intakePistons, conveyor, rightKicker, leftKicker, linebreakSensor,
            drivetrain);
    private final TopBallOut topBallOut = new TopBallOut(conveyor, rightKicker, leftKicker, linebreakSensor, shooter);
    private final BottomBallOut bottomBallOut = new BottomBallOut(intake, intakePistons, conveyor);
    private final ShootVision shootVision = new ShootVision(conveyor, rightKicker, leftKicker, linebreakSensor, shooter,
            limelight);
    private final LockTurret lockTurret = new LockTurret(turret, limelight, drivetrain);
    private final ThrottleTurret throttleTurret = new ThrottleTurret(turret, xb::getLeftX);
    private final SpinUpShooter spinUpShooter = new SpinUpShooter(shooter, limelight);
    private final FireWhenReady fireWhenReady = new FireWhenReady(conveyor, leftKicker, rightKicker, shooter);

    // Auto Paths
    private final RamseteCommandWrapper first1 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("first 1"), Constants.AutoConstants.RAMSETE_CONSTANTS);
    private final RamseteCommandWrapper first2 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("first 2"), Constants.AutoConstants.RAMSETE_CONSTANTS);
    private final RamseteCommandWrapper first3 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("first 3"), Constants.AutoConstants.RAMSETE_CONSTANTS);
    private final RamseteCommandWrapper first4 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("first 4"), Constants.AutoConstants.RAMSETE_CONSTANTS);
    private final RamseteCommandWrapper second3 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("second 3"), Constants.AutoConstants.RAMSETE_CONSTANTS);
    private final RamseteCommandWrapper second4 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("second 4"), Constants.AutoConstants.RAMSETE_CONSTANTS);
    private final RamseteCommandWrapper third3 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("third 3"), Constants.AutoConstants.RAMSETE_CONSTANTS);
    private final RamseteCommandWrapper third4 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("third 4"), Constants.AutoConstants.RAMSETE_CONSTANTS);
    private final RamseteCommandWrapper firstSteal1 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("first steal 1"), Constants.AutoConstants.RAMSETE_CONSTANTS);
    private final RamseteCommandWrapper firstSteal2 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("first steal 2"), Constants.AutoConstants.RAMSETE_CONSTANTS);
    private final RamseteCommandWrapper secondSteal = new RamseteCommandWrapper(drivetrain,
            new AutoPath("second steal"), Constants.AutoConstants.RAMSETE_CONSTANTS);
    private final RamseteCommandWrapper stash = new RamseteCommandWrapper(drivetrain,
            new AutoPath("stash"), Constants.AutoConstants.RAMSETE_CONSTANTS);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Set default commands
        drivetrain.setDefaultCommand(arcadeDrive);
        // drivetrain.setDefaultCommand(tankDrive);
        turret.setDefaultCommand(throttleTurret);
        // Driver Station configuration
        // DriverStation.silenceJoystickConnectionWarning(true);
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Instantiate button bindings

        JoystickButton aButton = new JoystickButton(xb, DeadbandXboxController.Button.kA.value);
        JoystickButton bButton = new JoystickButton(xb, DeadbandXboxController.Button.kB.value);
        JoystickButton xButton = new JoystickButton(xb, DeadbandXboxController.Button.kX.value);
        JoystickButton yButton = new JoystickButton(xb, DeadbandXboxController.Button.kY.value);

        // Xbox Controller POV buttons
        POVButton dpadUp = new POVButton(xb, 0);
        POVButton dpadDown = new POVButton(xb, 180);

        // Xbox Controller triggers
        Trigger leftTrigger = new Trigger(xb::getLeftTrigger);
        Trigger rightTrigger = new Trigger(xb::getLeftTrigger);

        // Xbox Controller bumpers
        Trigger leftBumper = new JoystickButton(xb, DeadbandXboxController.Button.kLeftBumper.value);

        // Xbox Controller sticks
        Trigger leftStickY = new Trigger(() -> {
            return xb.getLeftY() != 0;
        });
        Trigger leftStickX = new Trigger(() -> {
            return xb.getLeftX() != 0;
        });
        Trigger rightStickY = new Trigger(() -> {
            return xb.getLeftY() != 0;
        });
        Trigger rightStickX = new Trigger(() -> {
            return xb.getLeftX() != 0;
        });

        Trigger limelightTrigger = new Trigger(limelight::hasTarget);

        limelightTrigger.whenActive(lockTurret);

        /**
         * INTAKE
         */
        bButton.and(leftTrigger.negate())
                .whileActiveOnce(index);
        bButton.and(leftTrigger)
                .whileActiveOnce(bottomBallOut);

        yButton.and(leftTrigger)
                .whileActiveOnce(topBallOut);

        aButton.whenActive(spinUpShooter);
        leftBumper.whenActive(fireWhenReady);


        /**
         * SHOOTER
         */
        xButton
                .whileHeld(shootVision);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * 
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // TODO: add targeting code!!!!!
        Command pos1 = new SequentialCommandGroup(
                new ParallelCommandGroup(index, first1.setOdometryToFirstPoseOnStart()),
                shootVision,
                new ParallelCommandGroup(index, firstSteal1),
                new ParallelCommandGroup(index, secondSteal),
                stash,
                topBallOut,
                bottomBallOut);

        Command pos2 = new SequentialCommandGroup(
                new ParallelCommandGroup(index, first2.setOdometryToFirstPoseOnStart()),
                shootVision,
                new ParallelCommandGroup(index, firstSteal2),
                new ParallelCommandGroup(index, secondSteal),
                stash,
                topBallOut,
                bottomBallOut);

        Command pos3 = new SequentialCommandGroup(
                new ParallelCommandGroup(index, first3.setOdometryToFirstPoseOnStart()),
                new ParallelCommandGroup(index, second3),
                shootVision,
                new ParallelCommandGroup(index, third3),
                shootVision);

        Command pos4 = new SequentialCommandGroup(
                new ParallelCommandGroup(index, first4.setOdometryToFirstPoseOnStart()),
                new ParallelCommandGroup(index, second4),
                shootVision,
                new ParallelCommandGroup(index, third4),
                shootVision);

        return pos1;
    }
}