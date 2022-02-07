// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.Indexer;

public class Tuck extends CommandBase {
  private Feed m_feed;
  private Indexer m_indexer;
  private boolean m_isTuck;
  /** Creates a new Tuck. */
  public Tuck(Feed subsystem, Indexer subsystem2, boolean IsTuck) {
    m_feed = subsystem;
    m_indexer = subsystem2;
    m_isTuck = IsTuck;

  
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_isTuck){
      m_feed.feedRun(.5);
      m_indexer.indexRun();
    }else{
      m_feed.feedReverse();
      m_indexer.indexReverse();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feed.stopFeed();
    m_indexer.stopIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
