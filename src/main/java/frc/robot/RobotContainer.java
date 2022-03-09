// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5587.lib.auto.AutoPath;
import org.frc5587.lib.auto.RamseteCommandWrapper;
import org.frc5587.lib.control.*;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  private final DeadbandXboxController xb = new DeadbandXboxController(1, 1.5);
  
  // Subsystems
  private final Drivetrain drivetrain = new Drivetrain();
  private final Intake intake = new Intake();
  private final IntakePistons intakePistons = new IntakePistons();
  private final Conveyor conveyor = new Conveyor();
  private final Kicker rightKicker = Kicker.createRightKicker();
  private final Kicker leftKicker = Kicker.createLeftKicker();
  private final LinebreakSensor linebreakSensor = new LinebreakSensor();
  private final Shooter shooter = new Shooter();
  private final Limelight limelight = new Limelight();

  // Commands
  private final ArcadeDrive arcadeDrive = new ArcadeDrive(drivetrain, joystick::getY, () -> -joystick.getXCurveDampened());
  // private final TankDrive tankDrive = new TankDrive(drivetrain, joystick::getY, rightJoystick::getY);
  private final Index index = new Index(intake, intakePistons, conveyor, rightKicker, leftKicker, linebreakSensor);
  private final TopBallOut topBallOut = new TopBallOut(conveyor, rightKicker, leftKicker, linebreakSensor, shooter);
  private final BottomBallOut bottomBallOut = new BottomBallOut(intake, intakePistons, conveyor);
  private final ShootDashboard shootDashboard = new ShootDashboard(shooter, rightKicker, leftKicker, shooter::getSmartDashboard);
  private final ShootOne shootOne = new ShootOne(conveyor, rightKicker, leftKicker, linebreakSensor, shooter, shooter::getSmartDashboard);
  private final ShootVision shootVision = new ShootVision(shooter, limelight);
  private final MoveDown moveDown = new MoveDown(intake, intakePistons, conveyor, rightKicker, leftKicker, linebreakSensor);
  private final IntakeOnly intakeOnly = new IntakeOnly(intake, intakePistons);
  
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
    // TODO Sendable chooser to choose drive command (ArcadeDrive or TankDrive)
    drivetrain.setDefaultCommand(arcadeDrive);
    // drivetrain.setDefaultCommand(tankDrive);
    
    // Driver Station configuration
    DriverStation.silenceJoystickConnectionWarning(true);
    // Configure the button bindings
    configureButtonBindings();
    // Configure the default commands
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

      /*
      Shooter
      */
      // leftStickY
      //   .whileActiveOnce(new ShootBasic(shooter, xb::getLeftY));
      // leftStickY
      //   .whileActiveOnce(new ShootBasic(shooter, shooter.getSmartDashboard()));
      xButton
        .whileHeld(shootOne);

    // Xbox Controller POV buttons
    POVButton dpadUp = new POVButton(xb, 0);
    POVButton dpadDown = new POVButton(xb, 180);

    // Xbox Controller triggers
    Trigger leftTrigger = new Trigger(() -> xb.getLeftTrigger());
    Trigger rightTrigger = new Trigger(() -> xb.getLeftTrigger());

    // Xbox Controller sticks
    Trigger leftStickY = new Trigger(() -> { 
      return xb.getLeftY() != 0;
    });
    Trigger rightStickY = new Trigger(() -> {
      return xb.getLeftY() != 0;
    });

    
    /**
     * INTAKE
     */
    aButton.and(leftTrigger.negate())
      .whileActiveOnce(index);

    // yButton.and(leftTrigger.negate())
    //   .whileActiveOnce(moveDown);

    bButton.and(leftTrigger.negate())
      .whileActiveOnce(intakeOnly);
    dpadUp
      .whenHeld(new TopBallOut(conveyor, rightKicker, leftKicker, linebreakSensor, shooter));
      
    dpadDown
      .whenHeld(bottomBallOut);


    /**
     * SHOOTER
     */
    // xButton
    //   .whenHeld(shootDashboard);
    xButton
      .whenHeld(shootVision);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //TODO: add targeting code!!!!!
    Command pos1 = new SequentialCommandGroup(
      new ParallelCommandGroup(index, first1.setOdometryToFirstPoseOnStart()),
      shootOne,
      new ParallelCommandGroup(index, firstSteal1),
      new ParallelCommandGroup(index, secondSteal),
      stash,
      topBallOut,
      bottomBallOut
    );

    Command pos2 = new SequentialCommandGroup(
      new ParallelCommandGroup(index, first2.setOdometryToFirstPoseOnStart()),
      shootOne,
      new ParallelCommandGroup(index, firstSteal2),
      new ParallelCommandGroup(index, secondSteal),
      stash,
      topBallOut,
      bottomBallOut
    );

    Command pos3 = new SequentialCommandGroup(
      new ParallelCommandGroup(index, first3.setOdometryToFirstPoseOnStart()),
      new ParallelCommandGroup(index, second3),
      shootOne,
      new ParallelCommandGroup(index, third3),
      shootOne
    );

    Command pos4 = new SequentialCommandGroup(
      new ParallelCommandGroup(index, first4.setOdometryToFirstPoseOnStart()),
      new ParallelCommandGroup(index, second4),
      shootOne,
      new ParallelCommandGroup(index, third4),
      shootOne
    );

    return pos1;
  }
}