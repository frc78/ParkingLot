// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase {
  private static TalonFX shooterWheel = new TalonFX(Constants.LeftShoot);
  private static TalonFX shooterWheel2 = new TalonFX(Constants.RightShoot);
  private static TalonFXConfiguration _velocity_closed = new TalonFXConfiguration();


  /**NOT NEEDED YET public static int shootMode = 0;*/

  /** Creates a new Shooter. */
  public Shooter() {
    //reset the motor configurations
    shooterWheel.configFactoryDefault();
    shooterWheel2.configFactoryDefault();

    shooterWheel2.follow(shooterWheel);
    shooterWheel2.configNeutralDeadband(0);// this basically makes it so that shooter two never gets ahead and that they are always gonna be together :]❤

    shooterWheel.setInverted(TalonFXInvertType.CounterClockwise);//This will be subject to change if needed or if it needs to change direction
    shooterWheel.setSensorPhase(false);
    shooterWheel2.setInverted(TalonFXInvertType.Clockwise);

    shooterWheel.setNeutralMode(NeutralMode.Coast);
    shooterWheel2.setNeutralMode(NeutralMode.Coast);

    
    // Config all PID settings
    _velocity_closed.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    _velocity_closed.nominalOutputForward = 0;
    _velocity_closed.nominalOutputReverse = 0;
    _velocity_closed.peakOutputForward = 1;
    _velocity_closed.peakOutputReverse = 0; // Should never go in reverse
    _velocity_closed.slot0.kF = 0.04843;//0.05;
    _velocity_closed.slot0.kP = 0.078;//0.02;
    _velocity_closed.slot0.kI = 0;
    _velocity_closed.slot0.kD = 0;

    shooterWheel.configAllSettings(_velocity_closed);
    shooterWheel.selectProfileSlot(0, 0);
  }
 public void startWheely(){
   shooterWheel.set(ControlMode.PercentOutput, 0.33);
   shooterWheel2.set(ControlMode.PercentOutput, 0.33);
 }

 public void startWheel(double velocity){
  velocity = ((velocity * 2048) / 600); // Convert velocity in RPM to Units Per 100ms
  shooterWheel.set(ControlMode.Velocity, velocity);
}

 public void stopWheely(){
   shooterWheel.set(ControlMode.PercentOutput, 0);
   shooterWheel2.set(ControlMode.PercentOutput, 0);
 }
  @Override
  public void periodic() {
    if (Constants.DEBUG) {
      SmartDashboard.putNumber("Actual Velocity", (shooterWheel.getSelectedSensorVelocity() / 2048 * (Math.PI * 6)));
      SmartDashboard.putNumber("Target Velocity", shooterWheel.getClosedLoopTarget());
    }
    //double vel = SmartDashboard.getNumber("Velocity", Constants.spinupVel);
    //double vel2 = SmartDashboard.getNumber("VelocityTop", Constants.spinupVel2);
    // This method will be called once per scheduler run
  }
}
