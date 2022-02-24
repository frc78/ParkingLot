// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Auto.AutoStraight;
import frc.robot.commands.Auto.AutoTurn;
import frc.robot.commands.Auto.PathCommands;
import frc.robot.commands.Auto.Auto.AUTO1BALLSEQ;
import frc.robot.commands.Auto.Auto.Auto1Ball2Par;
import frc.robot.commands.Auto.Auto.Auto1Ball3Par;
import frc.robot.commands.Auto.Auto.Auto2BallSEQ;
import frc.robot.commands.Auto.Auto.AutoTaxi1Seq;
import frc.robot.commands.Auto.Auto.BackupAutoTestSeq;
import frc.robot.commands.Auto.Auto.Testing;
import frc.robot.commands.Auto.PathweaverAutos.Auto1BallPar;
import frc.robot.commands.Auto.PathweaverAutos.Auto1BallSeq;
import frc.robot.commands.Auto.PathweaverAutos.AutoTestSeq;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Drive.Forward50;
import frc.robot.commands.Drive.Tank;
import frc.robot.commands.Hanger.DeployHanger;
import frc.robot.commands.Hanger.HangerOverrideReset;
import frc.robot.commands.Hanger.SetUpHanger;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Intake.IntakeNoFeed;
import frc.robot.commands.Intake.Tuck;
import frc.robot.commands.Shoot.SpinUp;
import frc.robot.commands.Shoot.Fire;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.FeedWheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Chassis.Chassis;
import frc.robot.subsystems.Chassis.Hanger;
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
  private final ThreeMotorChassis m_chassis;
  private final Intake m_intake;
  private final Shooter m_shooter;
  private final Feed m_feed;
  private final Indexer m_indexer;
  private final FeedWheel m_feedWheel;
  private final Hanger m_hanger;

  //THERE IS PROBABLY A BETTER WAY TO DO THIS THAN INSTANTIATING A CLASS
  private final PathCommands m_pathcommands;

  //            JOYSTICKS
  private final XboxController m_driveController;
 
  private final Joystick m_manipController; 
  
  //  Shooter
  // private final Shooter m_shooter;
  


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    //CameraServer.startAutomaticCapture();
    m_intake = new Intake();
    m_chassis = new ThreeMotorChassis();
    m_shooter = new Shooter();
    m_feed = new Feed();
    m_indexer = new Indexer();
    m_feedWheel = new FeedWheel();
    m_hanger = new Hanger();
    // m_shooter = new Shooter();
    m_driveController = new XboxController(Constants.DRIVEJS);
    m_manipController = new Joystick(Constants.DRIVEMP);
    m_pathcommands = new PathCommands();
    
  
    //Configure the button bindings
    configureButtonBindings();
    SmartDashboard.putData(new HangerOverrideReset(m_hanger));


    m_chassis.setDefaultCommand(new Tank(m_chassis, m_driveController));

    m_intake.setDefaultCommand(new PerpetualCommand(new InstantCommand(m_intake::StopIntake, m_intake)));

    UsbCamera RobotCamera = CameraServer.startAutomaticCapture();
    RobotCamera.setResolution(128, 72); 

    /*RobotCamera.setQuality(50);
    	CameraServer.startAutomaticCapture("cam0");*/

    m_shooter.setDefaultCommand(new PerpetualCommand(new InstantCommand(m_shooter::stopWheely, m_shooter)));
    m_feed.setDefaultCommand(new PerpetualCommand(new InstantCommand(m_feed::stopFeed, m_feed)));
    m_feedWheel.setDefaultCommand(new PerpetualCommand(new InstantCommand(m_feedWheel::stopFeedWheel, m_feedWheel)));
    m_hanger.setDefaultCommand(new DeployHanger(m_hanger, m_manipController));
    //m_hanger.setDefaultCommand(new InstantCommand(m_hanger::hover));
    
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
    
    Button driveControllerSTART = new JoystickButton(m_driveController, 10);
    driveControllerSTART.whileHeld(new SetUpHanger(m_hanger));

    Button manipControllerA = new JoystickButton(m_manipController, 1);//Bandaid fix
    manipControllerA.whenHeld(new IntakeNoFeed(m_intake, m_indexer));

    Button manipControllerX = new JoystickButton(m_manipController, 3);// swapped with x until wiring is fixed
    manipControllerX.whileHeld(new IntakeCommand(m_intake, m_feed, m_indexer, false));

    Button manipControllerRB = new JoystickButton(m_manipController, 6);
    manipControllerRB.whileHeld(new Fire(m_feed, m_indexer, m_feedWheel));

    Button manipControllerRT = new JoystickButton(m_manipController, 10);
    manipControllerRT.whileHeld(new InstantCommand(m_hanger::hover, m_hanger));

    Button manipControllerLB = new JoystickButton(m_manipController, 5);
    manipControllerLB.whileHeld(new SpinUp(m_shooter, Constants.spinupVel2));//High Goal

    Button manipControllerLowLT = new JoystickButton(m_manipController, 7);
    manipControllerLowLT.whileHeld(new SpinUp(m_shooter, Constants.spinupVel));//Low goal
    
    Button manipControllerB = new JoystickButton(m_manipController, 2);// swapped with b 
    manipControllerB.whileHeld(new Tuck(m_feed, m_indexer, true));

    Button manipControllerY = new JoystickButton(m_manipController, 4);//bandaid---- Fixed Officially Names and everything. 
    manipControllerY.whileHeld(new Tuck(m_feed, m_indexer, false));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // return new BackupAuto1Seq(m_chassis);
    // return new AutoTaxi1Seq(m_chassis, m_indexer, m_feed, m_feedWheel);
  //  return new Auto1Ball2Par(m_intake, m_feed, m_indexer, m_shooter, m_chassis);
    // return new Auto1Ball3Par(m_intake, m_feed, m_indexer, m_shooter, m_chassis, m_feedWheel);
    return new AUTO1BALLSEQ(m_chassis, m_feed, m_indexer, m_shooter, m_feedWheel);     //AUTO 1 BALL
    //return new Auto2BallSEQ(m_chassis, m_intake, m_feed, m_shooter, m_feedWheel, m_indexer); // AUTO 2 BALL



   //return new Testing(m_chassis);
    
  }
}