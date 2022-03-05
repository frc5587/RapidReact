// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5587.lib.auto.AutoPath;
import org.frc5587.lib.auto.RamseteCommandWrapper;
import org.frc5587.lib.control.*;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
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

  // Commands
  private final ArcadeDrive arcadeDrive = new ArcadeDrive(drivetrain, joystick::getY, () -> -joystick.getXCurveDampened());
  // private final TankDrive tankDrive = new TankDrive(drivetrain, joystick::getY, rightJoystick::getY);
  private final RamseteCommandWrapper pickUpBall = new RamseteCommandWrapper(drivetrain,
    new AutoPath("pick up ball red 1"), Constants.AutoConstants.RAMSETE_CONSTANTS);

  // private final RamseteCommandWrapper goToLaunchpad = new RamseteCommandWrapper(drivetrain,
  //   new AutoPath("two meters"), Constants.AutoConstants.RAMSETE_CONSTANTS);
  
  // Others

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
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton yButton = new JoystickButton(xb, DeadbandXboxController.Button.kY.value);
    Trigger rightTrigger = new Trigger(xb::getLeftTrigger);

    yButton.and(rightTrigger).whenActive(() -> {
      drivetrain.setOdometry(
        new Pose2d(new Translation2d(0, 2), new Rotation2d(0))
      );
      System.out.println("IM DOIN THE THING!!!!!");
    });
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command shootFromLP = new SequentialCommandGroup(pickUpBall.setOdometryToFirstPoseOnStart());
    return shootFromLP;
    // return null;
  }
}