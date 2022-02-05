// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class Unjam extends CommandBase {
  public Intake m_unjam;

  /** Creates a new Unjam. */
  public Unjam(Intake subsystem) {
    m_unjam = subsystem; 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_unjam);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_unjam.DeployIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_unjam.setSpeed(0.75);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void setDefaultCommand(InstantCommand instantCommand) {
  }
}
