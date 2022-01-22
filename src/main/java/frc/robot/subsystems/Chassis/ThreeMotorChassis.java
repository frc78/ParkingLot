// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Chassis;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants;

/** Add your docs here. */
public class ThreeMotorChassis extends Chassis {

    // Drive Motors
    private TalonFX leftFollower1;
    private TalonFX leftFollower2;

    private TalonFX rightFollower1;
    private TalonFX rightFollower2;

    public ThreeMotorChassis() {
        super();
        
        // Set the motor IDs
        leftFollower1 = new TalonFX(Constants.LFOLLOWER);
        leftFollower2 = new TalonFX(Constants.L2FOLLOWER);

        rightFollower1 = new TalonFX(Constants.RFOLLOWER);
        rightFollower2 = new TalonFX(Constants.R2FOLLOWER);

        // Set Follower
        leftFollower1.follow(leftLeader);
        leftFollower2.follow(leftLeader);

        rightFollower1.follow(rightLeader);
        rightFollower2.follow(rightLeader);

        // Set inverted
        leftFollower1.setInverted(Constants.LEFT_INVERTED);
        leftFollower2.setInverted(Constants.LEFT_INVERTED);

        rightFollower1.setInverted(!Constants.LEFT_INVERTED);
        rightFollower2.setInverted(!Constants.LEFT_INVERTED);

    }
}
