// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Chassis;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants;

/** Add your docs here. */
public class ThreeMotorChassis extends Chassis {

    // Drive Motors
    private WPI_TalonFX leftFollower1;
    private WPI_TalonFX leftFollower2;

    private WPI_TalonFX rightFollower1;
    private WPI_TalonFX rightFollower2;

    public ThreeMotorChassis() {
        super();
        
        // Set the motor IDs
        leftFollower1 = new WPI_TalonFX(Constants.LFOLLOWER);
        leftFollower2 = new WPI_TalonFX(Constants.L2FOLLOWER);

        rightFollower1 = new WPI_TalonFX(Constants.RFOLLOWER);
        rightFollower2 = new WPI_TalonFX(Constants.R2FOLLOWER);

        // Reset motors
        leftFollower1.configFactoryDefault();
        leftFollower2.configFactoryDefault();

        rightFollower1.configFactoryDefault();
        rightFollower2.configFactoryDefault();

        // Set Follower
        leftFollower1.follow(leftLeader);
        leftFollower2.follow(leftLeader);

        rightFollower1.follow(rightLeader);
        rightFollower2.follow(rightLeader);

        // Set inverted
        leftFollower1.setInverted(InvertType.FollowMaster);
        leftFollower2.setInverted(InvertType.FollowMaster);

        rightFollower1.setInverted(InvertType.FollowMaster);
        rightFollower2.setInverted(InvertType.FollowMaster);

        // Set motor mode
        leftFollower1.setNeutralMode(Constants.MOTOR_MODE);
        leftFollower2.setNeutralMode(Constants.MOTOR_MODE);

        rightFollower1.setNeutralMode(Constants.MOTOR_MODE);
        rightFollower2.setNeutralMode(Constants.MOTOR_MODE);

        // Set minimum time to ramp up (originally .8)
        leftFollower1.configOpenloopRamp(0.78);
        leftFollower2.configOpenloopRamp(0.78);

        rightFollower1.configOpenloopRamp(0.78);
        rightFollower2.configOpenloopRamp(0.78);
    }
}
