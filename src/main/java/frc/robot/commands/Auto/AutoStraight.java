// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis.Chassis;

public class AutoStraight extends CommandBase {

  private Chassis m_chassis;
  private double encDistance;
  private double distance;
  private double speed;
  private PIDController PID;
  //private double goalDistance;
  
  public AutoStraight(Chassis subsytem1, double distanceM, double maxSpeed) {
    m_chassis = subsytem1;
    speed = maxSpeed;
    distance = distanceM;
    PID = new PIDController(Constants.kP, Constants.kI, Constants.kD);
    //encDistance = (int) Math.round(((distance / Constants.WHEEL_CIRC_METERS) / Constants.WHEEL_GEAR_RATIO) * Constants.UNITS_PER_REVOLUTION);
    encDistance = ((distanceM / Constants.WHEEL_CIRC_METERS) * Constants.WHEEL_GEAR_RATIO) * Constants.UNITS_PER_REVOLUTION;
    addRequirements(m_chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_chassis.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_chassis.setSpeed(calcSpeed(m_chassis.getRawMotorPosition(0), 1, encDistance), calcSpeed(m_chassis.getRawMotorPosition(0), 1, encDistance));
    // m_chassis.setSpeed(calcSpeed(m_chassis.getRawMotorPosition(0), speed, encDistance), calcSpeed(m_chassis.getRawMotorPosition(0), speed, encDistance));
    // m_chassis.setSpeed(speed, speed);
    m_chassis.setSpeed(PID.calculate(m_chassis.getMotorPositionExt(0), distance), PID.calculate(m_chassis.getMotorPositionExt(1), distance));
    //DriverStation.reportError("enc position :" + m_chassis.getRawMotorPosition(0), false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_chassis.setSpeed(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return encDistance < Math.abs(m_chassis.getRawMotorPosition(0)) ? true:false;
  }

  // this just doesn't seem to work for now, would be nice to fix it
  // double calcSpeed(double enc, double maxSpeed, double maxDistance) {
  //   if (enc < (maxDistance * 0.25)){
  //     return maxSpeed * (enc / (maxDistance * 0.25));
  //   } else if (enc > (maxDistance * 0.75)){
  //     return maxSpeed * (1 - ((enc - (maxDistance * 0.75)) / (maxDistance * 0.25)));
  //   } else {
  //     return maxSpeed;
  //   }
  // }
}
