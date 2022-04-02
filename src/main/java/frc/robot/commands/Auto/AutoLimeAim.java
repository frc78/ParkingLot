// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Chassis.Chassis;

public class AutoLimeAim extends CommandBase {
  /** Creates a new AutoLimeAim. */
  private Chassis m_chassis;
  private Limelight m_limelight;

  private double topspeed;
  private double threshold;
  public AutoLimeAim(Chassis chassis, Limelight limelight, double topspeed, double threshold) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_chassis = chassis;
    m_limelight = limelight;
    this.topspeed = topspeed;
    this.threshold = threshold;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_chassis.setSpeed(limelightAiming(topspeed, threshold, true), limelightAiming(topspeed, threshold, false));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_chassis.setSpeed(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_limelight.getTarget("x")) < threshold ? true : false;
  }

  private double limelightAiming (double topSpeed, double thres, boolean side) {
    if (m_limelight.hasTarget()) {
      double x = Math.abs(m_limelight.getTarget("x")) < thres ? 0 : m_limelight.getTarget("x");
      double degRange = 8;
  
      return ((Math.min(Math.abs(x), degRange) / degRange) * topSpeed * (side==true ? 1:-1) * (Math.abs(x) / x));
    } else {
      return side ? topspeed : topspeed * -1;
    }
  }
}
