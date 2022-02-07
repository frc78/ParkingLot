// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {
  /** Creates a new Intake. */
  private Intake m_intake;
  private Feed m_feed;
  private Indexer m_indexer;
  private Boolean m_isIntake;
  

  public IntakeCommand(Intake subsystem, Feed subsystem2, Indexer subsystem3, boolean IsIntake) {

    m_intake = subsystem;
    m_feed = subsystem2;
    m_indexer = subsystem3;
    m_isIntake = IsIntake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake, m_feed, m_indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.DeployIntake();
    m_feed.feedRun(0.0);
    m_indexer.indexRun();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_isIntake){
      m_intake.DeployIntake();
      m_intake.setSpeed(-.75);
      m_feed.feedRun(.75);
      m_indexer.indexRun();
    }else{
      m_intake.DeployIntake();
      m_intake.setSpeed(.75);
    }

    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.StopIntake();
    m_feed.stopFeed();
    m_indexer.stopIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
