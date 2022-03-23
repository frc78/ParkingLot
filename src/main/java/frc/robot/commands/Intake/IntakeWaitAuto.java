// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class IntakeWaitAuto extends CommandBase {
  private Intake m_intake;
  private Feed m_feed;
  private Indexer m_indexer;
  private double m_waitTime;
  private double startTime;
  /** Creates a new IntakeWaitAuto. */
  public IntakeWaitAuto(Intake intake, Feed feed, Indexer indexer, double waitTime) {
    m_intake = intake;
    m_feed = feed;
    m_indexer = indexer;
    m_waitTime = waitTime;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake, m_feed, m_indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.DeployIntake();
    m_feed.feedRun(1);
    m_indexer.indexRun();
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feed.feedRun(0);
    m_indexer.stopIndexer();
    m_intake.StopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return(Timer.getFPGATimestamp() > startTime + m_waitTime);
  }
}
