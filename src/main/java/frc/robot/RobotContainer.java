// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5587.lib.control.DeadbandXboxController;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
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
  // The robot's subsystems and commands are defined here...
  private final Conveyor conveyor = new Conveyor();
  private final Kicker kicker = new Kicker();

  private final DeadbandXboxController xboxController = new DeadbandXboxController(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
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
    JoystickButton aButton = new JoystickButton(xboxController, XboxController.Button.kA.value);
    JoystickButton xButton = new JoystickButton(xboxController, XboxController.Button.kX.value);

    Trigger leftTrigger = new Trigger(() -> xboxController.getLeftTriggerAxis() > 0);

    POVButton dpadUp = new POVButton(xboxController, 90);
    POVButton dpadDown = new POVButton(xboxController, 180);

    /**
     * CONVEYOR
     */
    // TODO Discuss logic flow of control 
    // "a" button spins conveyor motors at a speed of 0.2
    aButton.whenActive(() -> conveyor.setVelocity(0.2)).whenInactive(() -> conveyor.setVelocity(0));
    // "dpadUp" button spins conveyor motors at a speed of 1
    dpadUp.whenActive(() -> conveyor.setVelocity(1)).whenInactive(() -> conveyor.setVelocity(0));

    // "a" button and the left trigger spins conveyor motors at a speed of -0.2 (reverse)
    aButton.and(leftTrigger).whenActive(() -> conveyor.setVelocity(-0.2)).whenInactive(() -> conveyor.setVelocity(0));
    // "dpadUp" button and the left trigger spins conveyor motors at a speed of -1 (reverse)
    dpadUp.and(leftTrigger).whenActive(() -> conveyor.setVelocity(-1)).whenInactive(() -> conveyor.setVelocity(0));

    /**
     * KICKER
     */
    // "x" button spins kicker motors at a speed of 0.2
    xButton.whenActive(() -> kicker.setVelocity(0.2)).whenInactive(() -> kicker.setVelocity(0));
    // "dpadUp" button spins kicker motors at a speed of 1
    dpadDown.whenActive(() -> kicker.setVelocity(1)).whenInactive(() -> kicker.setVelocity(0));

    // "x" button and the left trigger spins kicker motors at a speed of -0.2 (reverse)
    xButton.and(leftTrigger).whenActive(() -> kicker.setVelocity(-0.2)).whenInactive(() -> kicker.setVelocity(0));
    // "dpadUp" button and the left trigger spins kicker motors at a speed of -1 (reverse)
    dpadDown.and(leftTrigger).whenActive(() -> kicker.setVelocity(-1)).whenInactive(() -> kicker.setVelocity(0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
