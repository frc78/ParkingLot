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

    public static final boolean DEBUG = false;

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

    public static final int backWheel1 = 15;
    public static final int backWheel2 = 16;

    //Indexer
    public static final int Indexer = 12;//Temporary subject to change.

    //Shooter Velocity
    public static final double spinupVel = 2168.0; //Bottom Goal
    public static final double spinupVel2 = 1725.0;//Top Goal
    public static final double spinUpVel3 = 800.0;//top fender
    public static final double spinUpVel4 = 3600.0;// far shot
    //            JOYSTICKS
    public static final int DRIVEJS = 1;
    public static final int DRIVEMP = 0;

    //            SENSORS
    public static final int IMU = 0;

    //            PARAMETERS FOR LARGE CHASSIS
    public static final TalonFXInvertType LEFT_INVERTED = TalonFXInvertType.CounterClockwise;
    public static final TalonFXInvertType RIGHT_INVERTED = TalonFXInvertType.Clockwise;
    public static final NeutralMode MOTOR_MODE = NeutralMode.Coast;
    public static final int UNITS_PER_REVOLUTION = 2048;
    public static final double WHEEL_CIRC_METERS = 0.47877;
    public static final double WHEEL_GEAR_RATIO = 12.27;

    public static final double kTrackWidthMeters = 0.6985;
    //            LIMELIGHT
    // height of the lime light from the ground, in meters
    // public static final double LIME_HEIGHTM = 0.1397; // small chassis
    // // x axis angle of limelight from the horizontal plane
    // public static final double LIME_ANGLE = 511;
    // height of the high goal (8ft 8in, 264cm) in meters
    public static final double HIGHGOAL_HEIGHTM = 2.416; //2.416 is real goal height
    // the optimal shooting distance from the high goal for shooting into the high goal, roughly around tarmac line, in meters
    public static final double OPTIMAL_SHOOTING_DISTANCEM = 2;
    // offsets for the x and y angle crosshair of the limelight, changing where the target should be for optimal shooting, in degs
    public static final double X_CROSS_OFFSET = 0;
    public static final double Y_CROSS_OFFSET = 0;

    //large chassis
    public static final double LIME_HEIGHTM = 0.762;
    public static final double LIME_ANGLE = 59.198; // old is 65.64 //old 60.612 // 1.143 to goal, 1.27 del hight

    //            PATH FOLLOWING CONSTANTS
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);

    //        Feedback OLD
    // public static final double ksVolts = 0.22;
    // public static final double kvVoltSecondsPerMeter = 1.98;
    // public static final double kaVoltSecondsSquaredPerMeter = 0.2;
    // public static final double kPDriveVel = 8.5;
    // NEWER FEEDBACK
    public static final double ksVolts = 0.621;
    public static final double kvVoltSecondsPerMeter = 4.1176;
    public static final double kaVoltSecondsSquaredPerMeter = 0.32188;
    public static final double kPDriveVel = 0.9399;

    //        Robot Speed
    public static final double kMaxSpeedMetersPerSecond = 1.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.75;
    //        Ramsete Controller
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    // PID
    public static final double kP = 9; // 16
    // public static final double kP = 2.7;
    public static final double kI = 0.7;
    public static final double kD = 0.1; // maybe put them back to 0 later perhaps

    // //            PARAMETERS FOR SMALL CHASSIS
    // public static final TalonFXInvertType LEFT_INVERTED = TalonFXInvertType.CounterClockwise;
    // public static final TalonFXInvertType RIGHT_INVERTED = TalonFXInvertType.Clockwise;
    // public static final NeutralMode MOTOR_MODE = NeutralMode.Coast;
    // public static final int UNITS_PER_REVOLUTION = 2048;
    // public static final double WHEEL_CIRC_METERS = 0.478;
    // public static final double WHEEL_GEAR_RATIO = 9.091;

    // //            PATH FOLLOWING CONSTANTS
    // public static final double kTrackWidthMeters = 0.584;
    // public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);

    // //        Feedback
    // public static final double ksVolts = 0.22;
    // public static final double kvVoltSecondsPerMeter = 1.98;
    // public static final double kaVoltSecondsSquaredPerMeter = 0.2;
    // public static final double kPDriveVel = 8.5;

    // //        Robot Speed
    // public static final double kMaxSpeedMetersPerSecond = 2.5;
    // public static final double kMaxAccelerationMetersPerSecondSquared = 2.5;
    // //        Ramsete Controller
    // public static final double kRamseteB = 2;
    // public static final double kRamseteZeta = 0.7;

    // // PID
    // public static final double kP = 10;
    // // public static final double kP = 2.7;
    // public static final double kI = 0.05;
    // public static final double kD = 0.05; // maybe put them back to 0 later perhaps
}
