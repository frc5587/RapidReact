// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ClimbConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import org.frc5587.lib.control.*;

import org.frc5587.lib.auto.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final Climb outerLeftClimb = Climb.createInnerRightArm();
    private final Climb outerRightClimb = Climb.createOuterRightArm();
    private final Climb innerLeftClimb = Climb.createInnerLeftArm();
    private final Climb innerRightClimb = Climb.createOuterLeftArm();
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
    private final ShootVision shootVision = new ShootVision(conveyor, rightKicker, leftKicker, linebreakSensor, shooter,
            limelight);
    private final LockTurret lockTurret = new LockTurret(turret, limelight, drivetrain);
    private final ThrottleTurret throttleTurret = new ThrottleTurret(turret, xb);
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
    private final RamseteCommandWrapper firstshoot4 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("firstshoot 4"), Constants.AutoConstants.RAMSETE_CONSTANTS);
    private final RamseteCommandWrapper finalshoot3 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("finalshoot 3"), Constants.AutoConstants.RAMSETE_CONSTANTS);
    private final RamseteCommandWrapper finalshoot4 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("finalshoot 4"), Constants.AutoConstants.RAMSETE_CONSTANTS);
    private final RamseteCommandWrapper firstSteal1 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("first steal 1"), Constants.AutoConstants.RAMSETE_CONSTANTS);
    private final RamseteCommandWrapper firstSteal2 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("first steal 2"), Constants.AutoConstants.RAMSETE_CONSTANTS);
    private final RamseteCommandWrapper secondSteal = new RamseteCommandWrapper(drivetrain,
            new AutoPath("second steal"), Constants.AutoConstants.RAMSETE_CONSTANTS);
    private final RamseteCommandWrapper stash = new RamseteCommandWrapper(drivetrain,
            new AutoPath("stash"), Constants.AutoConstants.RAMSETE_CONSTANTS);
    // define auto command groups here so they can be referenced anywhere
    private Command pos1;
    private Command pos2;
    private Command pos3;
    private Command pos4;

    // Other
    SendableChooser<Command> autoChooser = new SendableChooser<Command>();


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Set default commands
        drivetrain.setDefaultCommand(arcadeDrive);
        // drivetrain.setDefaultCommand(tankDrive);
        JoystickButton bButton = new JoystickButton(xb, DeadbandXboxController.Button.kB.value);
        turret.setDefaultCommand(throttleTurret);
        // Driver Station configuration
        // DriverStation.silenceJoystickConnectionWarning(true);
        // Add autonomous commands
        buildAutos();
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


        /**
         * SHOOTER
         */
        // xButton
        //         .whileHeld(shootVision);

        // Climb
        
    xb.aButton.and(xb.rightTrigger).whenActive(new ToggleClimbPistons(climbPistons));
    xb.bButton.and(xb.rightTrigger).whenActive(new SameClimbPistons(climbPistons, true));
    xb.xButton.and(xb.rightTrigger).whenActive(new SameClimbPistons(climbPistons, false));

    xb.leftStickY.and(xb.rightTrigger)
        .whileActiveOnce(new ClimbThrottle(outerLeftClimb, outerRightClimb, xb::getLeftY));

    xb.rightStickX.and(xb.rightTrigger)
        .whileActiveOnce(new ClimbThrottle(innerLeftClimb, innerRightClimb, xb::getRightY));

    xb.dPadDown.and(xb.rightTrigger)
        .whileActiveOnce(new SequentialCommandGroup(
            new ParallelCommandGroup(
                new ClimbToPosition(outerLeftClimb, outerRightClimb, ClimbConstants.UPPER_lIMIT, false),
                new ClimbToPosition(innerLeftClimb, innerRightClimb, ClimbConstants.UPPER_lIMIT, false)
                // ,new ToggleClimbPistons(climbPistons)
                )));

    xb.dPadLeft.and(xb.rightTrigger)
        .whileActiveOnce(new ClimbToPosition(outerLeftClimb, outerRightClimb, ClimbConstants.LOWER_LIMIT, true));

    xb.dPadUp.and(xb.rightTrigger).whileActiveOnce(new SequentialCommandGroup(
      new ClimbToPosition(innerLeftClimb, innerRightClimb, ClimbConstants.LOWER_LIMIT, true),
      new WaitCommand(0.5),
      new ClimbToPosition(outerLeftClimb, outerRightClimb, ClimbConstants.UPPER_lIMIT, true)));
    }

    public void buildAutos() {
        this.pos1 = new ParallelCommandGroup(
                lockTurret,
                new SequentialCommandGroup(
                        new ParallelCommandGroup(index, first1.setOdometryToFirstPoseOnStart()),
                        new ParallelRaceGroup(new WaitCommand(6), shootVision),
                        new ParallelCommandGroup(index, firstSteal1),
                        new ParallelCommandGroup(index, secondSteal),
                        stash,
                        topBallOut,
                        bottomBallOut
                )
        );

        this.pos2 = new ParallelCommandGroup(
                lockTurret,
                new SequentialCommandGroup(
                        new ParallelCommandGroup(index, first2.setOdometryToFirstPoseOnStart()),
                        new ParallelRaceGroup(new WaitCommand(6), shootVision),
                        new ParallelCommandGroup(index, firstSteal2),
                        new ParallelCommandGroup(index, secondSteal),
                        stash,
                        topBallOut,
                        bottomBallOut
                )
        );

        this.pos3 = new ParallelCommandGroup(
                lockTurret,
                new SequentialCommandGroup(
                        new ParallelCommandGroup(index, first3.setOdometryToFirstPoseOnStart()),
                        new ParallelRaceGroup(new WaitCommand(6), shootVision),
                        new ParallelCommandGroup(index, second3),
                        new ParallelCommandGroup(index, third3),
                        finalshoot3,
                        new ParallelRaceGroup(new WaitCommand(6), shootVision)
                )
        );

        this.pos4 = new ParallelCommandGroup(
                lockTurret,
                new SequentialCommandGroup(
                        new ParallelCommandGroup(index, first4.setOdometryToFirstPoseOnStart()),
                        firstshoot4,
                        new ParallelRaceGroup(new WaitCommand(6), shootVision),
                        new ParallelCommandGroup(index, second4),
                        new ParallelCommandGroup(index, third4),
                        finalshoot4,
                        new ParallelRaceGroup(new WaitCommand(6), shootVision)
                )
        );

        autoChooser.addOption("Position 1", pos1);
        autoChooser.addOption("Position 2", pos2);
        autoChooser.addOption("Position 3", pos3);
        autoChooser.addOption("Position 4", pos4);
        autoChooser.setDefaultOption("Field Position 1", pos1);
        SmartDashboard.putData(autoChooser);
    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * 
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
        // return pos1;
    }
}