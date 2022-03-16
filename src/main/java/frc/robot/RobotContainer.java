// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

import org.frc5587.lib.control.*;

import edu.wpi.first.wpilibj.*;
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
    private final DeadbandXboxController xb = new DeadbandXboxController(1);

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
    private final Climb outerLeftClimb = Climb.createOuterLeftArm();
    private final Climb outerRightClimb = Climb.createOuterRightArm();
    private final Climb innerLeftClimb = Climb.createInnerLeftArm();
    private final Climb innerRightClimb = Climb.createInnerRightArm();
    private final ClimbPistons climbPistons = new ClimbPistons();

    // Commands
    private final ArcadeDrive arcadeDrive = new ArcadeDrive(drivetrain, joystick::getY,
            () -> -joystick.getXCurveDampened());
    // private final TankDrive tankDrive = new TankDrive(drivetrain, joystick::getY,
    // rightJoystick::getY);
    private final Index index = new Index(intake, intakePistons, conveyor, rightKicker, leftKicker, linebreakSensor,
            drivetrain);
    private final TopBallOut topBallOut = new TopBallOut(conveyor, rightKicker, leftKicker, linebreakSensor, shooter);
    private final BottomBallOut bottomBallOut = new BottomBallOut(intake, intakePistons, conveyor);
    private final LockTurret lockTurret = new LockTurret(turret, limelight, drivetrain);
    private final ThrottleTurret throttleTurret = new ThrottleTurret(turret, xb);
    private final SpinUpShooter spinUpShooter = new SpinUpShooter(shooter, limelight);
    private final FireWhenReady fireWhenReady = new FireWhenReady(conveyor, leftKicker, rightKicker, shooter);

    // Auto Paths
    private AutoPaths autopaths = new AutoPaths(intake, intakePistons, conveyor, rightKicker, leftKicker, linebreakSensor,
        drivetrain, limelight, turret, shooter);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Set default commands
        drivetrain.setDefaultCommand(arcadeDrive);
        // drivetrain.setDefaultCommand(tankDrive);
        turret.setDefaultCommand(throttleTurret);
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
        Trigger limelightTrigger = new Trigger(limelight::hasTarget);

        limelightTrigger.whileActiveOnce(lockTurret);

        // leftStickX.whileActiveOnce(throttleTurret).negate().whileActiveOnce(lockTurret);
// 
        /**
         * INTAKE
         */
        xb.bButton.and(xb.leftTrigger.negate())
                .whileActiveOnce(index);
        xb.bButton.and(xb.leftTrigger)
                .whileActiveOnce(bottomBallOut);

        xb.yButton.and(xb.leftTrigger)
                .whileActiveOnce(topBallOut);

        xb.aButton.whileActiveOnce(spinUpShooter);
        xb.leftBumper.whileActiveOnce(fireWhenReady);


        // dpadUp
        //         .whileActiveOnce(new TestKicker(rightKicker, leftKicker));

        /**
         * SHOOTER
         */
        // xButton
        //         .whileHeld(shootVision);

        // Climb
        
//     aButton.and(rightTrigger).whenActive(new ToggleClimbPistons(climbPistons));
//     bButton.and(rightTrigger).whenActive(new SameClimbPistons(climbPistons, true));
//     xButton.and(rightTrigger).whenActive(new SameClimbPistons(climbPistons, false));

//     leftStickY.and(rightTrigger)
//         .whileActiveOnce(new ClimbThrottle(outerLeftClimb, outerRightClimb, xb::getLeftY));

//     rightStickX.and(rightTrigger)
//         .whileActiveOnce(new ClimbThrottle(innerLeftClimb, innerRightClimb, xb::getRightY));

//     dpadDown.and(rightTrigger)
//         .whileActiveOnce(new SequentialCommandGroup(
//             new ParallelCommandGroup(
//                 new ClimbToPosition(outerLeftClimb, outerRightClimb, ClimbConstants.UPPER_lIMIT, false),
//                 new ClimbToPosition(innerLeftClimb, innerRightClimb, ClimbConstants.UPPER_lIMIT, false)
//                 // ,new ToggleClimbPistons(climbPistons)
//                 )));

//     dpadLeft.and(rightTrigger)
//         .whileActiveOnce(new ClimbToPosition(outerLeftClimb, outerRightClimb, ClimbConstants.LOWER_LIMIT, true));

//     dpadUp.and(rightTrigger).whileActiveOnce(new SequentialCommandGroup(
//       new ClimbToPosition(innerLeftClimb, innerRightClimb, ClimbConstants.LOWER_LIMIT, true),
//       new WaitCommand(0.5),
//       new ClimbToPosition(outerLeftClimb, outerRightClimb, ClimbConstants.UPPER_lIMIT, true)));

        xb.rightTrigger.and(xb.rightStickY).whileActiveOnce(new ClimbThrottle(innerLeftClimb, innerRightClimb, outerLeftClimb, outerRightClimb, xb::getRightY));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * 
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // buildAutos();
        return autopaths.getSelectedCommand();
        // return new ParallelCommandGroup(
        //         new SequentialCommandGroup(
        //                 new ParallelRaceGroup(
        //                         first1.setOdometryToFirstPoseOnStart(), 
        //                         new Index(intake, intakePistons, conveyor, rightKicker, leftKicker, linebreakSensor, drivetrain)
        //                 ), 
        //                 new ParallelCommandGroup(
        //                         new SpinUpShooter(shooter, limelight),
        //                         new FireWhenReady(conveyor, leftKicker, rightKicker, shooter)
        //                 )
        //         ), 
        // new LockTurret(turret, limelight, drivetrain));
        // return pos1;
    }
}