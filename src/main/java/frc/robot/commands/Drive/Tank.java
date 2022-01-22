// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import java.util.ResourceBundle.Control;

import edu.wpi.first.wpilibj.Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis.Chassis;
import frc.robot.subsystems.Chassis.ThreeMotorChassis;

public class Tank extends CommandBase {

  private Chassis m_chassis;
  private XboxController m_controller;

  /** Creates a new Tank. */
  public Tank(Chassis chassis, XboxController xbox) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_chassis = chassis;
    m_controller = xbox;
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //sets the speed variables, passes in 0.3 as the maxAdjust arguement so that the default speed is 0.7 and range will be 0.4 to 1
    //and the 3rd arguemtent is for speed smoothing, 0 is for no smoothing, 1 is for left motor and 2 is for right motor
    double lSpeed = triggerAdjustedSpeed(m_controller.getLeftY(), 0.3, 0);
    double rSpeed = triggerAdjustedSpeed(m_controller.getRightY(), 0.3, 0);

    m_chassis.setSpeed(lSpeed, rSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_chassis.setSpeed(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  //this function is meant to take in the joystick input, and maxAdjust has to be between 0 and 1, 
  //THIS FUNCTION HASN'T BEEN TESTED YET
  public double triggerAdjustedSpeed(double inputSpeed, double maxAdjust, int smoothing){
    double tempSpeed = m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis();
    //tempSpeed = (tempSpeed * maxAdjust) + (inputSpeed * (1 - maxAdjust));
    tempSpeed = ((tempSpeed * maxAdjust) + 1) * (inputSpeed * (1 - maxAdjust));
    
    // i think there is a 50% chance this works. so be carful testing this lol
    // if smoothing = 1 that is the left motor, so it measures the left motor speed for the speed smoothing, if 2 it means the right
    //if m_chassis was of type ThreeMotorChassis, you need to use the getMotorSpeeds indexes 0 and 3 for leftleader and rightleader
    if(smoothing == 1){
      //uses a ternary (short hand) if statement to check what class m_chassis is (either ThreeMotorChassis or Chassis),
      //and uses different indexes of the getmotorspeedsarray, since in Chassis the leaders are 0 and 1
      //and in ThreeMotorChassis they are 0 and 3
      tempSpeed = tempSpeed + ((tempSpeed - ((m_chassis.getClass() == ThreeMotorChassis.class) ? m_chassis.getMotorSpeeds()[0] : m_chassis.getMotorSpeeds()[0])) / 2);
    } else if (smoothing == 2) {
      tempSpeed = tempSpeed + ((tempSpeed - ((m_chassis.getClass() == ThreeMotorChassis.class) ? m_chassis.getMotorSpeeds()[3] : m_chassis.getMotorSpeeds()[1])) / 2);
    }
    

    //clamping the output to make sure it doesnt exit -1 and 1 (for saftey)
    return Math.max(-1, Math.min(1, tempSpeed));
  }
}
