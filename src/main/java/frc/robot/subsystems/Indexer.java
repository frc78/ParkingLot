// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Indexer extends SubsystemBase {
  private TalonFX IndexMotor = new TalonFX(Constants.Indexer);
  
  /** Creates a new Indexer. */
  public Indexer() {
    IndexMotor.configFactoryDefault();
    IndexMotor.setNeutralMode(NeutralMode.Coast);
  
  }
  public void indexRun(){
    IndexMotor.set(ControlMode.PercentOutput, 0.75);
  }
  public void indexReverse(){
    IndexMotor.set(ControlMode.PercentOutput, -0.75);
  }

  public void stopIndexer() {
    IndexMotor.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }
}
