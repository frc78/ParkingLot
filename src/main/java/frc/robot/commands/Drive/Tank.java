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
    double lSpeed = triggerAdjustedSpeed(m_controller.getLeftY(), 0.5);
    double rSpeed = triggerAdjustedSpeed(m_controller.getRightY(), 0.5);

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

  //this function is meant to take in the joystick input, and maxAdjust has to be between 0 and 1
  public double triggerAdjustedSpeed(double inputSpeed, double maxAdjust){
    double tempSpeed = m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis();
    tempSpeed = ((1 - maxAdjust) * inputSpeed) + (inputSpeed * maxAdjust * tempSpeed);

    //clamping the output to make sure it doesnt exit -1 and 1 (for saftey)
    return Math.max(-1, Math.min(1, tempSpeed));
  }
}
