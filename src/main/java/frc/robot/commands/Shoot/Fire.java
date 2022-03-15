// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.FeedWheel;

public class Fire extends CommandBase {
  private Feed m_feed;
  private Indexer m_indexer;
  private FeedWheel m_feedWheel;
  private Intake m_intake;
  /** Creates a new RunFeed. */
  public Fire(Feed subsystem, Indexer subsystem2, FeedWheel subsystem3, Intake subsystem4) {
    m_feed = subsystem;
    m_indexer = subsystem2;
    m_feedWheel = subsystem3;
    m_intake = subsystem4;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_feed, m_indexer, m_feedWheel, m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.compressorOn(false);
    
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_indexer.indexRun();
    m_feed.feedRun(.6);
    m_feedWheel.runFeedWheel();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexer.stopIndexer();
    m_feed.stopFeed();
    m_feedWheel.stopFeedWheel();
    m_intake.compressorOn(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
