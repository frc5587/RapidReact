// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5587.lib.control.DeadbandXboxController;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Turret;

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

  private final Turret turret = new Turret();

  private final DeadbandXboxController xboxController = new DeadbandXboxController(1);

  // The robot's subsystems and commands are defined here...

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
    JoystickButton bButton = new JoystickButton(xboxController, XboxController.Button.kB.value);
    JoystickButton aButton = new JoystickButton(xboxController, XboxController.Button.kA.value);
    Trigger leftTrigger = new Trigger(() -> xboxController.getLeftTriggerAxis() > 0);

    bButton.whenActive(() -> turret.setMotorThrottle(xboxController.getLeftX())).whenInactive(() -> turret.stopTurret());
    // bButton.whenActive(() -> turret.setTurret(0.2)).whenInactive(() -> turret.stopTurret());
    // aButton.whenActive(() -> turret.setTurret(1)).whenInactive(() -> turret.stopTurret());

    bButton.and(leftTrigger).whenActive(() -> turret.setMotorThrottle(-xboxController.getLeftX())).whenInactive(() -> turret.stopTurret());
    // bButton.and(leftTrigger).whenActive(() -> turret.setTurret(-0.2)).whenInactive(() -> turret.stopTurret());
    // aButton.and(leftTrigger).whenActive(() -> turret.setTurret(-1)).whenInactive(() -> turret.stopTurret());
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
