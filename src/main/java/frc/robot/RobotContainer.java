// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5587.lib.auto.AutoPath;
import org.frc5587.lib.auto.RamseteCommandWrapper;
import org.frc5587.lib.control.*;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
  // Controllers
  private final DeadbandJoystick joystick = new DeadbandJoystick(0, 1.5);
  // private final DeadbandJoystick rightJoystick = new DeadbandJoystick(2, 1.5);
  private final DeadbandXboxController xb = new DeadbandXboxController(1, 1.5);
  
  // Subsystems
  private final Drivetrain drivetrain = new Drivetrain();
  private final Shooter shooter = new Shooter();
  private final Conveyor conveyor = new Conveyor();
  private final Kicker rightKicker = Kicker.createRightKicker();
  private final Kicker leftKicker = Kicker.createLeftKicker();
  private final Intake intake = new Intake();
  private final IntakePistons intakePistons = new IntakePistons();
  private final LinebreakSensor shooterSensor = new LinebreakSensor();

  // Commands
  private final ArcadeDrive arcadeDrive = new ArcadeDrive(drivetrain, joystick::getY, () -> -joystick.getXCurveDampened());
  // private final TankDrive tankDrive = new TankDrive(drivetrain, joystick::getY, rightJoystick::getY);
  private final IntakeIn intakeIn = new IntakeIn(intake, intakePistons, conveyor);
  private final RunKickerUp runKickerUp = new RunKickerUp(conveyor, rightKicker, leftKicker, shooterSensor);
  private final ShootBasic shootBasic = new ShootBasic(shooter, shooter.getSmartDashboard());
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

  // Other

  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // TODO Sendable chooser to choose drive command (ArcadeDrive or TankDrive)
    drivetrain.setDefaultCommand(arcadeDrive);
    // drivetrain.setDefaultCommand(tankDrive);
    
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
      Trigger leftTrigger = new Trigger(() -> xb.getLeftTriggerAxis() > 0);

      /*
      Shooter
      */
      // leftStickY
      //   .whileActiveOnce(new ShootBasic(shooter, xboxController::getLeftY));
      // leftStickY
      //   .whileActiveOnce(new ShootBasic(shooter, shooter.getSmartDashboard()));
      xButton
        .whileHeld(shootBasic);

    /**
     * INTAKE
     */
    aButton.and(leftTrigger.negate())
        .whileActiveOnce(intakeIn)
        .whenInactive(runKickerUp);
    aButton.and(leftTrigger)
        .whileActiveOnce(new IntakeOut(intake, intakePistons, conveyor));
    
    /**
     * Kicker
     */

    // yButton.and(leftTrigger.negate())
    //   .whileActiveOnce(new RunKickerUp(conveyor, rightKicker, leftKicker, shooterSensor));

    yButton.and(leftTrigger.negate())
      .whileActiveOnce(new KickerOnly(rightKicker, leftKicker, shooterSensor));

    /**
     * CONVEYOR
     */

    bButton.and(leftTrigger.negate()) 
      .whileActiveOnce(new RunConveyorUpVelocity(conveyor));
    
    // yButton.and(leftTrigger.negate())
    //   .whileActiveOnce(new RunConveyorUpPosition(conveyor));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //TODO: update shooting code when turret is done lol
    Command red1 = new SequentialCommandGroup(
      new ParallelCommandGroup(intakeIn, first1.setOdometryToFirstPoseOnStart()),
      runKickerUp,
      shootBasic, //change to targeting/limelight/shooting code!!!
      new ParallelCommandGroup(intakeIn, firstSteal1),
      runKickerUp,
      new ParallelCommandGroup(intakeIn, secondSteal),
      runKickerUp,
      stash,
      shootBasic //change to targeting/limelight/shooting code!!!
    );

    return red1;
  }
}