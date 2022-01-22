// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import frc.robot.utility.Gains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //            MOTORS
    //        Drive Motors
    // LEFT
    public static final int LLEADER = 1;
    public static final int LFOLLOWER = 2;
    public static final int L2FOLLOWER = 3;

    // RIGHT
    public static final int RLEADER = 4;
    public static final int RFOLLOWER = 5;
    public static final int R2FOLLOWER = 6;

    //            JOYSTICKS
    public static final int DRIVEJS = 0;

    //            SENSORS
    public static final int IMU = 0;

    //            PARAMETERS
    public static final TalonFXInvertType LEFT_INVERT = TalonFXInvertType.CounterClockwise;
    public static final TalonFXInvertType RIGHT_INVERT = TalonFXInvertType.Clockwise;

    public static final int SENSOR_UNITS_PER_ROTATION = 2048;
    public static final double NEUTRAL_DEADBAND = 0.001;
    public static final double TURN_UNITS_PER_DEGREE = 8192.0 / 360.0;
    public static final Gains GAINS_MOTOR_PRIFILE = new Gains( 1.0, 0.0, 0.0, 1023.0/6800.0, 400, 1.00 );
    public static final int PRIMARY_PID_SLOT = 0;
    public static final int AUX_PID_SLOT = 1;

}
