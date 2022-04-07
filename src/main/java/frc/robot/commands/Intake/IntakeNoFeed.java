// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;

public class IntakeNoFeed extends CommandBase {
  private Intake m_intake;
  private Indexer m_indexer;
  /** Creates a new IntakeNoFeed. */
  public IntakeNoFeed(Intake intake, Indexer indexer) {
    m_intake = intake;
    m_indexer = indexer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake, m_indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.DeployIntake();
    m_indexer.indexRun();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.DeployIntake();
    m_intake.setSpeed(-.75);
    m_indexer.indexRun();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.StopIntake();
    m_indexer.stopIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
