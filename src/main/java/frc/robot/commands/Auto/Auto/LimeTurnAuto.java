// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Chassis.Chassis;

public class LimeTurnAuto extends CommandBase {

  Chassis chassis;
  Limelight limelight;
  /** Creates a new LimeTurnAuto. */
  public LimeTurnAuto(Chassis chassis, Limelight limelight) {
    this.chassis = chassis;
    this.limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double lSpeed = limelightAiming(.5, 0.2, 5, false);
    double rSpeed = limelightAiming(-.5, 0.2, 5, true);
    chassis.setSpeed(lSpeed, rSpeed);
  }

  /**
   * @param inputspeed the driver input
   * @param topSpeed the top speed it should add
   * @param thres the threshold for when it considers the angle too small to bother moving
   * @param side which side this is being used for (true is left, false is right)
   * @return Adjusted speed
   */
  public double limelightAiming (double inputspeed, double topSpeed, double thres, boolean side) {
    if (limelight.hasTarget()) {
      double x = Math.abs(limelight.getTarget("x")) < thres ? 0 : limelight.getTarget("x");
      double degRange = 8;
  
      return ((Math.min(Math.abs(x), degRange) / degRange) * topSpeed * (side==true ? 1:-1) * (Math.abs(x) / x)) + inputspeed;
    } else {
      return inputspeed;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(limelight.getTarget("x")) < 5;
  }
}
