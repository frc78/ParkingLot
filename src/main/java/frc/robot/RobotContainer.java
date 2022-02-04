// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Drive.Forward50;
import frc.robot.commands.Drive.Tank;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Intake.Unjam;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Chassis.Chassis;
import frc.robot.subsystems.Chassis.ThreeMotorChassis;
import edu.wpi.first.wpilibj2.command.button.Button;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //            SUBSYSTEMS
  private final Chassis m_chassis;
  private final Intake m_intake;
  private final Unjam m_unjam;

  //            JOYSTICKS
  private final XboxController m_driveController;
 
  private final XboxController m_manipController; 
  
  //  Shooter
  // private final Shooter m_shooter;
  


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    CameraServer.startAutomaticCapture();
    m_intake = new Intake();
    m_chassis = new ThreeMotorChassis();
    m_unjam = new Unjam(m_intake);
    // m_shooter = new Shooter();
    m_driveController = new XboxController(Constants.DRIVEJS);
    m_manipController = new XboxController(Constants.DRIVEMP);
    
  
    //Configure the button bindings
    configureButtonBindings();


    m_chassis.setDefaultCommand(new Tank(m_chassis, m_driveController));

    // UsbCamera RobotCamera = CameraServer.startAutomaticCapture();
    // RobotCamera.setResolution(640, 480);
     
    m_intake.setDefaultCommand(new InstantCommand(m_intake::StopIntake, m_intake));

    m_unjam.setDefaultCommand(new InstantCommand());

  }

  /**       
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton aButton = new JoystickButton(m_driveController, 1);
    aButton.whenHeld(new Forward50(m_chassis, 0.5));

    JoystickButton bButton = new JoystickButton(m_driveController, 2);
    bButton.whenHeld(new Forward50(m_chassis, -0.5));

    Button manipControllerRB = new JoystickButton(m_manipController, 6);
    manipControllerRB.whileHeld(new IntakeCommand(m_intake));

    Button manipControllerUnJam = new JoystickButton(m_manipController, 3);
    manipControllerUnJam.whileHeld(new IntakeCommand(m_intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null; // m_autoCommand;
  }
}
