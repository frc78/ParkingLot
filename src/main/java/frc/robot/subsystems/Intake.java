// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Constants;
//import edu.wpi.first.wpilibj.XboxController;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends SubsystemBase {

  /** Creates a new Intake. */
  DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 3,2);
  Compressor comp = new Compressor(PneumaticsModuleType.REVPH);

    protected TalonFX intake;
   // private XboxController m_controller;


  public Intake() {

  intake = new TalonFX(Constants.INTAKE);

  }

  public boolean getDigitalInput(){
    
    if(/*digitalinput ==*/true)
    return true;
    return false;
  }


  public void setSpeed(double speed) {
    intake.set(ControlMode.PercentOutput, speed);
  }
  public void compressorOn(boolean isOn){
    if(isOn){
      comp.enableDigital();
    }else{
      comp.disable();
    }
  }

  public void autoIntakeStart(Intake intake, Indexer indexer) {
    intake.DeployIntake();
    intake.setSpeed(-.75);
    indexer.indexRun();
  }

  public void autoIntakeStop(Intake intake, Indexer indexer) {
    intake.StopIntake();
    indexer.stopIndexer();
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
