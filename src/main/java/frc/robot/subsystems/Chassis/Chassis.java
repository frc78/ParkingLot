// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Chassis;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Chassis extends SubsystemBase {

  // Drive Motors
  protected TalonFX leftLeader;

  protected TalonFX rightLeader;

  /** Creates a new Chassis. */
  public Chassis() {
    
        // Set the motor IDs
        leftLeader = new TalonFX(Constants.LLEADER);

        rightLeader = new TalonFX(Constants.RLEADER);

        // Set inverted
        leftLeader.setInverted(Constants.LEFT_INVERTED);

        rightLeader.setInverted(!Constants.LEFT_INVERTED);

        leftLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rightLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }

  public void setSpeed(double lSpeed, double rSpeed) {
        leftLeader.set(ControlMode.PercentOutput, lSpeed);
        rightLeader.set(ControlMode.PercentOutput, rSpeed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("left encoder", leftLeader.getSelectedSensorVelocity());
    SmartDashboard.putNumber("right encoder", rightLeader.getSelectedSensorVelocity());
    // This method will be called once per scheduler run
  }
}