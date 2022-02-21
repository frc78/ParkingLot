// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/** Add your docs here. */
public class Climber {

    private WPI_TalonFX climbLeader;
    private WPI_TalonFX climbFollower;
    public Climber(){
        climbFollower = new WPI_TalonFX(constants.CFOLLOWER);
        climbLeader = new WPI_TalonFX(constants.CLEADER);

    }
}
