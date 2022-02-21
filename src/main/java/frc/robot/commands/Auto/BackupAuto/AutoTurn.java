// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.BackupAuto;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis.Chassis;

public class AutoTurn extends CommandBase {
  
  private Chassis m_chassis;
  private PigeonIMU pigeon;
  private double degrees;
  private double speed;
  /** Creates a new AutoTurn. */
  public AutoTurn(Chassis subsystem1, PigeonIMU gyro, double turnDegrees, double turnSpeed) {
    m_chassis = subsystem1;
    pigeon = gyro;
    degrees = turnDegrees;
    speed = turnSpeed;
    addRequirements(m_chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pigeon.setYaw(0, 30);

    if (degrees > 0) {
      m_chassis.setSpeed(speed * -1, speed);
    } else {
      m_chassis.setSpeed(speed, speed * -1);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_chassis.setSpeed(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (degrees > 0) {
      return pigeon.getYaw() > degrees ? true:false;
    } else {
      return pigeon.getYaw() < degrees ? true:false;
    }
  }
}
