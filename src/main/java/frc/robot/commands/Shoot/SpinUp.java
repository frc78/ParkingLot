// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;

public class SpinUp extends CommandBase {
  private Shooter shooter;
  private double vel;
  private Limelight limelight;
  


  /** Creates a new Shoot. */
  public SpinUp(Shooter shooter, double Velocity, Limelight limelight) {
    this.shooter = shooter;
    this.vel = Velocity;
    this.limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting spin up!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    shooter.startWheel(adjustedVel());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopWheely();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public double adjustedVel() {
    return limelight.highGoalDistance() * Constants.distVelRatio + Constants.distVelOffset;
  }
}
