// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Hanger extends SubsystemBase {
  /** Creates a new Hanger. */

  protected TalonFX hangerMotor;

  public Hanger() {

    hangerMotor = new TalonFX(Constants.HANGERMOTOR);
    hangerMotor.setNeutralMode(NeutralMode.Brake);

  }

  public void setSpeed(double speed){
    hangerMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
