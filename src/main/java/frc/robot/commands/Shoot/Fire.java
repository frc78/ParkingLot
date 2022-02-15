// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.FeedWheel;
import frc.robot.subsystems.BallCount;

public class Fire extends CommandBase {
  private Feed m_feed;
  private Indexer m_indexer;
  private FeedWheel m_feedWheel;
  /** Creates a new RunFeed. */
  public Fire(Feed subsystem, Indexer subsystem2, FeedWheel subsystem3) {
    m_feed = subsystem;
    m_indexer = subsystem2;
    m_feedWheel = subsystem3;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_feed, m_indexer, m_feedWheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_indexer.indexRun();
    m_feed.feedRun(.75);
    m_feedWheel.runFeedWheel();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexer.stopIndexer();
    m_feed.stopFeed();
    m_feedWheel.stopFeedWheel();
    BallCount.resetBallCount();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
