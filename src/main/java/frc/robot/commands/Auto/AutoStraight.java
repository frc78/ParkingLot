// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.lang.invoke.ConstantCallSite;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Utility;
import frc.robot.subsystems.Chassis.Chassis;

public class AutoStraight extends CommandBase {

  private Chassis m_chassis;
  private double encDistance;
  private double distance;
  private double speed;
  private ProfiledPIDController PID;

  private TrapezoidProfile trapProfile;
  private TrapezoidProfile.Constraints trapConstraints;
  private TrapezoidProfile.State trapGoal;
  private TrapezoidProfile.State trapStart;
  //private double goalDistance;

  private double lastSpeed;
  private double lastTime;
  private SimpleMotorFeedforward feedforward;
  
  public AutoStraight(Chassis subsytem1, double distanceM, double maxSpeed) {
    m_chassis = subsytem1;
    speed = maxSpeed;
    distance = distanceM;
    
    //trapezoid motion profiling & PID variables
    trapConstraints = new TrapezoidProfile.Constraints(Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared);
    trapGoal = new TrapezoidProfile.State(distanceM, 0);
    trapStart = new TrapezoidProfile.State(0, 0);
    trapProfile = new TrapezoidProfile(trapConstraints, trapGoal, trapStart);
    PID = new ProfiledPIDController(Constants.kP, Constants.kI, Constants.kD, trapConstraints);

    lastSpeed = PID.getSetpoint().velocity;
    lastTime = Timer.getFPGATimestamp();
    feedforward = new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter);
    
    //encDistance = (int) Math.round(((distance / Constants.WHEEL_CIRC_METERS) / Constants.WHEEL_GEAR_RATIO) * Constants.UNITS_PER_REVOLUTION);
    encDistance = Utility.metersToEncoders(distanceM);
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
    // m_chassis.setSpeed(speed, speed);
    // m_chassis.setSpeed(PID.calculate(m_chassis.getMotorPositionExt(0), distance), PID.calculate(m_chassis.getMotorPositionExt(1), distance));
    // speed = PID.calculate(getAverageEnc(), trapGoal);
    goToPosition(distance);
    // m_chassis.setSpeed(speed, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_chassis.setSpeed(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return encDistance < Math.abs(m_chassis.getRawMotorPosition(0)) ? true:false;
    return PID.atGoal();
  }

  private double getAverageEnc() {
    return (m_chassis.getMotorPositionExt(0) * 0.5) + (m_chassis.getMotorPositionExt(1) * 0.5);
  }

  // Controls a simple motor's position using a SimpleMotorFeedforward
  // and a ProfiledPIDController
  public void goToPosition(double goalPosition) {
    double acceleration = (PID.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
    double speed = PID.calculate(m_chassis.getMotorPositionExt(0), goalPosition)
                    + feedforward.calculate(PID.getSetpoint().velocity, acceleration);
    m_chassis.setVoltage(speed, speed);
    lastSpeed = PID.getSetpoint().velocity;
    lastTime = Timer.getFPGATimestamp();
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
