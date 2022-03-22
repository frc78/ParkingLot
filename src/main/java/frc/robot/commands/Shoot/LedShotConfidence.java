// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LEDs.FridayNightLights;
import frc.robot.subsystems.LEDs.LedsIO;
import frc.robot.subsystems.LEDs.LedsIO.LedMode;

public class LedShotConfidence extends CommandBase {
  private LedsIO m_led;
  private FridayNightLights m_flashy;
  private Limelight m_limelight;

  /** Creates a new LedShotConfidence. */
  public LedShotConfidence(LedsIO led, FridayNightLights flashy, Limelight limelight) {
    m_led = led;
    m_flashy = flashy;
    m_limelight = limelight;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double confidence = m_limelight.getXConfidence();
    if(confidence >= 0.7){
      m_flashy.solid(Color.kGreen);
    }else{
      m_flashy.solid(Color.kBlue);
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
