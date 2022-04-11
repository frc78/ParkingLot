// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis.Chassis;

public class AutoTurn extends CommandBase {
  
  private Chassis m_chassis;
  private double degrees;
  private double speed;
  private double initDeg;

  /** Creates a new AutoTurn. */
  public 
  AutoTurn(Chassis subsystem1, double turndegrees, double turnSpeed) {
    m_chassis = subsystem1;
    degrees = turndegrees;
    speed = turnSpeed;
    addRequirements(m_chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initDeg = m_chassis.getPidgeonYaw();
    if (degrees > 0) {
      m_chassis.setSpeed(speed, speed * -1);
    } else {
      m_chassis.setSpeed(speed * -1, speed);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_chassis.feedDrive();
    DriverStation.reportError("this is the readed headings: " + m_chassis.getPidgeonYaw(), false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_chassis.setSpeed(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      SmartDashboard.putNumber("AUTO_DEG", Math.abs(m_chassis.getPidgeonYaw() - Math.abs(initDeg)));
      SmartDashboard.putBoolean("AUTO_TURNFINISHED", Math.abs(m_chassis.getPidgeonYaw() - Math.abs(initDeg)) > Math.abs(degrees));
      // if (degrees < 0) {
      //   return Math.abs(m_chassis.getPidgeonYaw() - initDeg) < degrees ? true:false;
      // } else {
      //   return Math.abs(m_chassis.getPidgeonYaw() - initDeg) > degrees ? true:false;
      // }
      return Math.abs(m_chassis.getPidgeonYaw() - initDeg) > Math.abs(degrees) ? true:false;
  }
}
