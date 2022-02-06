// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Intake extends SubsystemBase {

  /** Creates a new Intake. */
  DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0,1);
  Compressor comp = new Compressor(1, PneumaticsModuleType.REVPH);

    protected TalonFX intake;
   // private XboxController m_controller;


  public Intake() {

  intake = new TalonFX(Constants.INTAKE);

  }

  public void setSpeed(double speed) {
    intake.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void DeployIntake() {
    solenoid.set(Value.kForward);
  }

  public void StopIntake() {
    setSpeed(0);
    solenoid.set(Value.kReverse);
  }
}
