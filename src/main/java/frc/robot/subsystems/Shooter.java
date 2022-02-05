// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase {
  private static TalonFX shooterWheel = new TalonFX(Constants.LeftShoot);
  private static TalonFX shooterWheel2 = new TalonFX(Constants.RightShoot);
  /** Creates a new Shooter. */
  public Shooter() {
    //reset the motor configurations
    shooterWheel.configFactoryDefault();
    shooterWheel2.configFactoryDefault();

    shooterWheel2.follow(shooterWheel);
    shooterWheel2.configNeutralDeadband(0);// this basically makes it so that shooter two never gets ahead and that they are always gonna be together :]❤

    shooterWheel.setInverted(TalonFXInvertType.Clockwise);//This will be subject to change if needed or if it needs to change direction
    shooterWheel.setSensorPhase(false);
    shooterWheel2.setInverted(TalonFXInvertType.CounterClockwise);

    shooterWheel.setNeutralMode(NeutralMode.Coast);
    shooterWheel2.setNeutralMode(NeutralMode.Coast);

  }
 public void startWheely(){
   shooterWheel.set(ControlMode.PercentOutput, 0.33);
   shooterWheel2.set(ControlMode.PercentOutput, -0.33);
 }
 public void stopWheely(){
   shooterWheel.set(ControlMode.PercentOutput, 0);
   shooterWheel2.set(ControlMode.PercentOutput, 0);
 }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
