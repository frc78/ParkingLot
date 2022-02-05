// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Drive.Forward50;
import frc.robot.commands.Drive.Tank;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Intake.Unjam;
import frc.robot.commands.Shoot.SpinUp;
import frc.robot.commands.Shoot.Fire;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.FeedWheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
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
  private final Shooter m_shooter;
  private final Feed m_feed;
  private final Indexer m_indexer;
  private final FeedWheel m_feedWheel;

  //            JOYSTICKS
  private final XboxController m_driveController;
 
  private final XboxController m_manipController; 
  
  //  Shooter
  // private final Shooter m_shooter;
  


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // CameraServer.startAutomaticCapture();
    m_intake = new Intake();
    m_chassis = new ThreeMotorChassis();
    m_unjam = new Unjam(m_intake);
    m_shooter = new Shooter();
    m_feed = new Feed();
    m_indexer = new Indexer();
    m_feedWheel = new FeedWheel();
    // m_shooter = new Shooter();
    m_driveController = new XboxController(Constants.DRIVEJS);
    m_manipController = new XboxController(Constants.DRIVEMP);
    
  
    //Configure the button bindings
    configureButtonBindings();


    m_chassis.setDefaultCommand(new Tank(m_chassis, m_driveController));

    m_intake.setDefaultCommand(new PerpetualCommand(new InstantCommand(m_intake::StopIntake, m_intake)));

    // UsbCamera RobotCamera = CameraServer.startAutomaticCapture();
    // RobotCamera.setResolution(640, 480);
     
    

    m_unjam.setDefaultCommand(new InstantCommand());

    m_shooter.setDefaultCommand(new PerpetualCommand(new InstantCommand(m_shooter::stopWheely)));

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

    Button manipControllerX = new JoystickButton(m_manipController, 3);
    manipControllerX.whileHeld(new IntakeCommand(m_intake, m_feed, m_indexer));

    Button manipControllerRB = new JoystickButton(m_manipController, 6);

    Button manipControllerUnJam = new JoystickButton(m_manipController, 3);
    manipControllerUnJam.whileHeld(new Unjam(m_intake));

    Button manipControllerLB = new JoystickButton(m_manipController, 7);
    manipControllerLB.whileHeld(new SpinUp(m_shooter));
    manipControllerRB.whenPressed(new Fire(m_feed, m_indexer, m_feedWheel));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
    //   new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
    //   Constants.kDriveKinematics,
    //   10
    // );
    // These are not needed as they are retrieved from path json file
    // TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.kDriveKinematics).addConstraint(autoVoltageConstraint);

    Trajectory trajectory = new Trajectory();

    String trajectoryJSON = "test1.wpilib.json";

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    RamseteCommand ramseteCommand = new RamseteCommand(
      trajectory, 
      m_chassis::getPose, 
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta), 
      new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter), 
      Constants.kDriveKinematics, 
      m_chassis::getWheelSpeeds, 
      new PIDController(Constants.kPDriveVel, 0, 0), 
      new PIDController(Constants.kPDriveVel, 0, 0), 
      m_chassis::setVoltage, 
      m_chassis
    );

    m_chassis.resetOdometry(trajectory.getInitialPose());

    // An ExampleCommand will run in autonomous
    return ramseteCommand.andThen(() -> m_chassis.stop()); // m_autoCommand;
  }
}
