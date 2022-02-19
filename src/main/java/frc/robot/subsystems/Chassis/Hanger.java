// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Chassis;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class Hanger extends SubsystemBase{

    private WPI_TalonFX climbLeader;
    private WPI_TalonFX climbFollower;
    public Hanger(){
    climbFollower = new WPI_TalonFX(Constants.CFOLLOWER);
    climbLeader = new WPI_TalonFX(Constants.CLEADER);
    climbFollower.follow(climbLeader);
    }
    public void rise() {
        climbLeader.set(ControlMode.PercentOutput, 0.5);
    }
    public void fall(){
        climbLeader.set(ControlMode.PercentOutput, 0.5);
    }
    public void stop(){
        climbLeader.set(ControlMode.PercentOutput, 0);
    }
    @Override
    public void periodic(){}
}



