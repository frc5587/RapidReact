// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ClimbConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import org.frc5587.lib.auto.*;
import org.frc5587.lib.control.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
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
    private final ClimbController climbController = new ClimbController();

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
    private final ClimbThrottle climbThrottle = new ClimbThrottle(climbController, xb::getRightY, xb::getLeftY);

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
    private final RamseteCommandWrapper secondSteal2 = new RamseteCommandWrapper(drivetrain,
            new AutoPath("second steal"), Constants.AutoConstants.RAMSETE_CONSTANTS);
    private final RamseteCommandWrapper stash = new RamseteCommandWrapper(drivetrain,
            new AutoPath("stash"), Constants.AutoConstants.RAMSETE_CONSTANTS);
    private final RamseteCommandWrapper stash2 = new RamseteCommandWrapper(drivetrain,
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
        turret.setDefaultCommand(throttleTurret);
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

        (xb.rightStickY.or(xb.leftStickY)).and(xb.rightTrigger)
            .whileActiveOnce(climbThrottle);

    }

    private void buildAutos() {
        this.pos1 = new ParallelCommandGroup(
                new LockTurret(turret, limelight, drivetrain),
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new Index(intake, intakePistons, conveyor, rightKicker, 
                                        leftKicker, linebreakSensor, drivetrain), 
                                first1
                        ),
                        new SpinUpShooter(shooter, limelight),
                        new FireWhenReady(conveyor, rightKicker, leftKicker, shooter),
                        new ParallelCommandGroup(
                                new Index(intake, intakePistons, conveyor, rightKicker, 
                                        leftKicker, linebreakSensor, drivetrain), 
                                firstSteal1
                        ),
                        new ParallelCommandGroup(
                                new Index(intake, intakePistons, conveyor, rightKicker, 
                                        leftKicker, linebreakSensor, drivetrain),
                                secondSteal
                        ),
                        stash,
                        new TopBallOut(conveyor, rightKicker, leftKicker, linebreakSensor, shooter),
                        new BottomBallOut(intake, intakePistons, conveyor)
                )
        );

        this.pos2 = new ParallelCommandGroup(
                new LockTurret(turret, limelight, drivetrain),
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new Index(intake, intakePistons, conveyor, rightKicker, 
                                        leftKicker, linebreakSensor, drivetrain), 
                                first2
                        ),
                        new SpinUpShooter(shooter, limelight),
                        new FireWhenReady(conveyor, rightKicker, leftKicker, shooter),
                        new ParallelCommandGroup(
                                new Index(intake, intakePistons, conveyor, rightKicker, 
                                        leftKicker, linebreakSensor, drivetrain), 
                                firstSteal2
                        ),
                        new ParallelCommandGroup(
                                new Index(intake, intakePistons, conveyor, rightKicker, 
                                        leftKicker, linebreakSensor, drivetrain),
                                secondSteal2
                        ),
                        stash2,
                        new TopBallOut(conveyor, rightKicker, leftKicker, linebreakSensor, shooter),
                        new BottomBallOut(intake, intakePistons, conveyor)
                )
        );

        this.pos3 = new ParallelCommandGroup(
                new LockTurret(turret, limelight, drivetrain),
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new Index(intake, intakePistons, conveyor, rightKicker, 
                                        leftKicker, linebreakSensor, drivetrain), 
                                first3
                        ),                        new SpinUpShooter(shooter, limelight),
                        new FireWhenReady(conveyor, rightKicker, leftKicker, shooter),
                        new ParallelCommandGroup(
                                new Index(intake, intakePistons, conveyor, rightKicker, 
                                        leftKicker, linebreakSensor, drivetrain),
                                second3
                        ),
                        new ParallelCommandGroup(
                                new Index(intake, intakePistons, conveyor, rightKicker, 
                                        leftKicker, linebreakSensor, drivetrain),
                                third3
                        ),
                        finalshoot3,
                        new SpinUpShooter(shooter, limelight),
                        new FireWhenReady(conveyor, rightKicker, leftKicker, shooter)
                )
        );

        this.pos4 = new ParallelCommandGroup(
                new LockTurret(turret, limelight, drivetrain),
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new Index(intake, intakePistons, conveyor, rightKicker, 
                                        leftKicker, linebreakSensor, drivetrain),
                                first4
                        ),
                        firstshoot4,
                        new SpinUpShooter(shooter, limelight),
                        new FireWhenReady(conveyor, rightKicker, leftKicker, shooter),
                        new ParallelCommandGroup(
                                new Index(intake, intakePistons, conveyor, rightKicker, 
                                        leftKicker, linebreakSensor, drivetrain),
                                second4
                        ),
                        new ParallelCommandGroup(
                                new Index(intake, intakePistons, conveyor, rightKicker, 
                                        leftKicker, linebreakSensor, drivetrain), 
                                third4
                        ),
                        finalshoot4,
                        new SpinUpShooter(shooter, limelight),
                        new FireWhenReady(conveyor, rightKicker, leftKicker, shooter)
                )
        );

        autoChooser.addOption("1st Position", pos1);
        autoChooser.addOption("2nd Position", pos2);
        autoChooser.addOption("3rd Position", pos3);
        autoChooser.addOption("4th Position", pos4);
        autoChooser.setDefaultOption("1st Position", pos1);
        SmartDashboard.putData(autoChooser);
    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * 
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // buildAutos();
        return autoChooser.getSelected();
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