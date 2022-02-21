// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.FeedWheel;
import frc.robot.subsystems.Indexer;

public class FireAuto extends CommandBase {
  /** Creates a new FireAuto. */
  private Feed m_feed;
  private Indexer m_indexer;
  private FeedWheel m_feedWheel;
  private RelativeEncoder relativeEncoder;
  
  public FireAuto(Indexer subsystem1, Feed subsystem2, FeedWheel subsystem3) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_indexer = subsystem1;
    m_feed = subsystem2;
    m_feedWheel = subsystem3;
    relativeEncoder = m_feed.beltneo.getEncoder();
    
    addRequirements(m_feed, m_indexer, m_feedWheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    relativeEncoder.setPosition(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_indexer.indexRun();
    m_feed.feedRun();
    m_feedWheel.runFeedWheel();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return relativeEncoder.getPosition() > 1.0;
  }
}
