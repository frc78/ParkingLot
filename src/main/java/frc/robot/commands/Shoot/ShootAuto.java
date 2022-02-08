// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shoot;

import java.lang.module.ModuleDescriptor.Requires;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.FeedWheel;
import frc.robot.subsystems.Indexer;

public class ShootAuto extends CommandBase {
  /** Creates a new ShootAuto. */
  private Feed m_feed;
  private Indexer m_indexer;
  private FeedWheel m_feedWheel;

  public ShootAuto(Feed subsystem, Indexer subsystem2, FeedWheel subsystem3) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_feed = subsystem;
    m_indexer = subsystem2;
    m_feedWheel = subsystem3;
    addRequirements(m_feed, m_indexer, m_feedWheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_indexer.indexRun();
    m_feed.feedRun();
    m_feedWheel.runFeedWheel();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexer.stopIndexer();
    m_feed.stopFeed();
    m_feedWheel.stopFeedWheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    RelativeEncoder encoder = m_feed.beltneo.getEncoder();
    double clicks = encoder.getPosition();
    return clicks > 1.0;
  }
}
