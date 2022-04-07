// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hanger;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis.Hanger;

public class DeployHanger extends CommandBase {
  private Hanger m_hanger;
  private Joystick m_controller;
  private int currentEncoderCLicks = 205000;
  private int MAX_ENCODER_CLICKS = 205000;
  private int NEW_MAX_ENCODER_CLICKS = 245000;

  /** Creates a new DeployHanger. */
  public DeployHanger(Hanger hanger, Joystick controller ) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    m_hanger = hanger;
    addRequirements(hanger);
    m_controller = controller;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_hanger.deployHangerPneumatics(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_hanger.isHangerDeployed()){
      currentEncoderCLicks = NEW_MAX_ENCODER_CLICKS;
    }else{
      currentEncoderCLicks = MAX_ENCODER_CLICKS;
    }
    int dPadValue = m_controller.getPOV();
    SmartDashboard.putNumber("currentEncoderClicks", currentEncoderCLicks);
     SmartDashboard.putNumber("ClimbClicks", m_hanger.getPosition());
        if(m_hanger.getPosition() > -2500 && m_hanger.getPosition() < currentEncoderCLicks){
          //will run down buttons 
          //SmartDashboard.putNumber("Climbif", dPadValue);
          if (dPadValue == -1){
            m_hanger.stop();
          } else if ((dPadValue > 90) && (dPadValue <= 270)){
            m_hanger.fall();
          } else if ((dPadValue <= 90) || (dPadValue > 270)){
            m_hanger.rise();
          }
        }else if(m_hanger.getPosition() <= -2500){
          //SmartDashboard.putNumber("Climbif", 2);
          //will not run down buttons
          if (dPadValue == -1){
            m_hanger.stop();
          } else if ((dPadValue > 90) && (dPadValue <= 270)){
            m_hanger.stop();
          } else if ((dPadValue <= 90) || (dPadValue > 270)){
            m_hanger.rise();
          }
        }else if(m_hanger.getPosition() > currentEncoderCLicks){
          //SmartDashboard.putNumber("Climbif", 3);
          // Buttons up will not run
          if (dPadValue == -1){
            m_hanger.stop();
          } else if ((dPadValue > 90) && (dPadValue <= 270)){
            m_hanger.fall();
          } else if ((dPadValue <= 90) || (dPadValue > 270)){
            m_hanger.stop();
          }
        }else{
          //SmartDashboard.putNumber("Climbif", 4);
          //do not run whatsoever
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
