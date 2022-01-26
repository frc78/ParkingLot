// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hanger;

public class Climb extends CommandBase {
  /** Creates a new Climb. */
  private XboxController m_controller;
  private Hanger m_hanger;

  public Climb(Hanger hanger, XboxController xbox) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hanger);
    
    m_hanger = hanger;
    m_controller = xbox;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_controller.getRightBumper()){
      m_hanger.setSpeed(1.0);
    } else if (m_controller.getLeftBumper()){
      m_hanger.setSpeed(-1.0);
    } else {
      m_hanger.setSpeed(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
