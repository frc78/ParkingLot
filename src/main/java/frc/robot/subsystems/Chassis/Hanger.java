// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Chassis;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Hanger.HangerOverrideReset;

/** Add your docs here. */
public class Hanger extends SubsystemBase{
    DoubleSolenoid solenoidHanger = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0,4);

    private WPI_TalonFX climbLeader;
    private WPI_TalonFX climbFollower;
    private double velocity = 3000; 
    private double hoverVelocity = 500;
    private double velocityRise = ((velocity * 2048) / 600);
    private double velocityFall = ((velocity * 2048) / 600 ) * -1 ;
    private double velocityHover = ((hoverVelocity * 2048) / 600);



    public Hanger(){

        climbFollower = new WPI_TalonFX(Constants.CFOLLOWER);
        climbLeader = new WPI_TalonFX(Constants.CLEADER);
        climbFollower.follow(climbLeader);

        climbFollower.setNeutralMode(NeutralMode.Brake);
        climbLeader.setNeutralMode(NeutralMode.Brake);
        climbFollower.setInverted(TalonFXInvertType.Clockwise);
        climbLeader.setInverted(TalonFXInvertType.CounterClockwise);

        climbLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        climbFollower.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        resetClimbEncoders();
        // climbFollower.follow(climbLeader);
    }
    public void resetClimbEncoders(){
        climbLeader.getSensorCollection().setIntegratedSensorPosition(0, 30);
        climbFollower.getSensorCollection().setIntegratedSensorPosition(0, 30);
    }

    public void rise() {
        climbLeader.set(ControlMode.PercentOutput, 1);
        climbFollower.set(ControlMode.PercentOutput, 1);
    }
    public void fall(){
        climbLeader.set(ControlMode.PercentOutput, -0.5);
        climbFollower.set(ControlMode.PercentOutput, -0.5);
    }
    public void slowFall() {
        climbLeader.set(ControlMode.PercentOutput, -0.2);
        climbFollower.set(ControlMode.PercentOutput, -0.2);
    }
    public void stop(){
        climbLeader.set(ControlMode.PercentOutput, 0);
        climbFollower.set(ControlMode.PercentOutput, 0);
    }
    public void deployHangerPneumatics(boolean isDeployed){
        if(isDeployed){
            solenoidHanger.set(Value.kForward);
          }else{
            solenoidHanger.set(Value.kReverse);
          }
    }
    public void hover(){
       // climbLeader.set(ControlMode.PercentOutput, 0.03);
        //climbFollower.set(ControlMode.PercentOutput, 0.03);
        climbLeader.setVoltage(-1.2);
        climbFollower.setVoltage(-1.2);
    }
    public double getPosition(){
       return climbLeader.getSelectedSensorPosition();
    }
    @Override
    public void periodic(){
     
    }
}



