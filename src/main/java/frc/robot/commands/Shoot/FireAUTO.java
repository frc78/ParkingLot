// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.FeedWheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class FireAUTO extends CommandBase {
  private Feed m_feed;
  private FeedWheel m_wheely;
  private Indexer m_index;
  private double startTime;
  private Intake m_intake;
  private double waitTime;
  /** Creates a new FireAUTO. */
  public FireAUTO(Feed feed, FeedWheel wheely, Indexer indexer, Intake intake, double shootTime) {
    m_feed = feed;
    m_wheely = wheely;
    m_index = indexer;
    m_intake = intake;
    waitTime = shootTime;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_feed, m_wheely, m_index, m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_index.indexRun();
    m_feed.feedRun(1);
    m_wheely.runFeedWheel();
    m_intake.compressorOn(false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_index.stopIndexer();
    m_wheely.stopFeedWheel();
    m_feed.stopFeed();
    m_intake.compressorOn(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

   return(Timer.getFPGATimestamp() > startTime + waitTime);
  }
}
