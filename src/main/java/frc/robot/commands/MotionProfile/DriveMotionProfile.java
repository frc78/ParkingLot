// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MotionProfile;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis.Chassis;
import frc.robot.utility.CTRE_Functions;
import frc.robot.utility.MotionProfiles.MotionProfile;

public class DriveMotionProfile extends CommandBase {

  private Chassis m_chassis;

  /** Creates a new DriveMotionProfile. */
  public DriveMotionProfile(Chassis chassis) {
    m_chassis = chassis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_chassis.zeroAllSensors();

    CTRE_Functions.initBuffer(MotionProfile.Points, MotionProfile.kNumPoints, 0.0, m_chassis.getTrajectoryStream());

    m_chassis.startMotionProfile();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { /* Do nothing here */ }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_chassis.getMotionProfileOver();
  }
}
