// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hanger;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis.Hanger;

public class DeployHanger extends CommandBase {
  private Hanger m_hanger;
  private XboxController m_controller;
  /** Creates a new DeployHanger. */
  public DeployHanger(Hanger hanger, XboxController controller ) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    m_hanger = hanger;
    addRequirements(hanger);
    m_controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        int dPadValue = m_controller.getPOV();
        if (dPadValue == -1){
          m_hanger.stop();
        } else if ((dPadValue > 90) && (dPadValue <= 270)){
          m_hanger.fall();
        } else if ((dPadValue <= 90) && (dPadValue > 270)){
          m_hanger.rise();
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
