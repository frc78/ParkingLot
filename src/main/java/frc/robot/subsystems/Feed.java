// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.print.CancelablePrintJob;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feed extends SubsystemBase {
  private static CANSparkMax beltneo = new CANSparkMax(Constants.BeltNeo, MotorType.kBrushless);
  private static CANSparkMax wheelNeo = new CANSparkMax(Constants.WheelNeo, MotorType.kBrushless);
  
  /** Creates a new Feed. */
  public Feed() {
   beltneo = new CANSparkMax(Constants.BeltNeo, null);
   wheelNeo = new CANSparkMax(Constants.WheelNeo, null);
   wheelNeo.follow(beltneo);//might take out idk yet.

  }
  public void SetFeed() {
    beltneo.set(.78);//subject to change during testing 
    wheelNeo.set(.78);//Also subject to change during testing

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
