// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

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

    //OTHER MOTORS
    //Climb Motors
    public static final int CLEADER = 13;
    public static final int CFOLLOWER = 14;

    //GREGS REQUEST
    public static final int INTAKE = 7;

    // Shooter
    public static final int LeftShoot = 8;// temporary value will fill in later. 
    public static final int RightShoot = 9;
    //Feed
    public static final int BeltNeo = 10;
    public static final int WheelNeo = 11;

    //Indexer
    public static final int Indexer = 12;//Temporary subject to change.
    //            JOYSTICKS
    public static final int DRIVEJS = 1;
    public static final int DRIVEMP = 0;

    //            SENSORS
    public static final int IMU = 0;

    //            PARAMETERS
    public static final TalonFXInvertType LEFT_INVERTED = TalonFXInvertType.CounterClockwise;
    public static final TalonFXInvertType RIGHT_INVERTED = TalonFXInvertType.Clockwise;
    public static final NeutralMode MOTOR_MODE = NeutralMode.Coast;
    public static final int UNITS_PER_REVOLUTION = 2048;
    public static final double WHEEL_CIRC_METERS = 0.478;
    public static final double WHEEL_GEAR_RATIO = 12.27;

    //            PATH FOLLOWING CONSTANTS
    public static final double kTrackWidthMeters = 0.584;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);

    //        Feedback
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;
    public static final double kPDriveVel = 8.5;

    //        Robot Speed
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    //        Ramsete Controller
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

}
