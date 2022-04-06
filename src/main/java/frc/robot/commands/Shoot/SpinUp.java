// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;

public class SpinUp extends CommandBase {
  private Shooter shooter;
  private double vel;
  private boolean isHood;
  private Feed feed;
  public double feedSpeed;
  public double bnt;


  /** Creates a new Shoot. */
  public SpinUp(Shooter shooter, double Velocity, Feed feed, double feedSpeed, double bnt, boolean isHood) {
    this.shooter = shooter;
    this.vel = Velocity;
    this.isHood = isHood;
    this.feed = feed;
    this.feedSpeed = feedSpeed;
    this.bnt = bnt;
    
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, feed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting spin up!");
    this.shooter.isHood(isHood);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    shooter.startWheel(vel);
    feed.feedRun(feedSpeed);
    //shooter.startBackWheels(shooter.getShooterSpeed() * bnt * 2);
    double neoPercent = vel * 5.93 * Math.PI * 0.75 / Math.PI / 6380;
    shooter.startBackWheels(neoPercent * bnt);

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
}
