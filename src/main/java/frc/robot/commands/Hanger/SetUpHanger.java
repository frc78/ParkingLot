// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hanger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis.Hanger;

public class SetUpHanger extends CommandBase {
  private Hanger m_hanger;

  /** Creates a new SetUpHanger. */
  public SetUpHanger(Hanger hanger) {
    m_hanger = hanger;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_hanger);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_hanger.getPosition() > 0 && m_hanger.getPosition() < 20000){
      m_hanger.rise();
    }else if(m_hanger.getPosition() >= 20000){
    m_hanger.stop();
    }else{
      m_hanger.stop();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hanger.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
