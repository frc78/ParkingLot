// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Chassis;

import java.io.Console;
import java.sql.Driver;
import java.util.concurrent.ExecutionException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utility;

public class Chassis extends SubsystemBase {

  // Drive Motors
  protected WPI_TalonFX leftLeader;

  protected WPI_TalonFX rightLeader;

  // Sensors
  protected WPI_PigeonIMU pidgey;

  // Setup drivetrain for auto
  private final DifferentialDrive m_drive;
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new Chassis. */
  public Chassis() {
    
    // Set the motor IDs
    leftLeader = new WPI_TalonFX(Constants.LLEADER);

    rightLeader = new WPI_TalonFX(Constants.RLEADER);

    // Reset motors
    leftLeader.configFactoryDefault();

    rightLeader.configFactoryDefault();

    // Set motor mode
    leftLeader.setNeutralMode(NeutralMode.Brake);

    rightLeader.setNeutralMode(NeutralMode.Brake);

    leftLeader.setSensorPhase(true);
    rightLeader.setSensorPhase(true);

    // Set inverted
    leftLeader.setInverted(Constants.LEFT_INVERTED);

    rightLeader.setInverted(Constants.RIGHT_INVERTED);

    leftLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    // Configure sensor
    pidgey = new WPI_PigeonIMU(Constants.IMU);

    // Reset encoders
    resetEncoder();

    // Setup drivebase
    m_drive = new DifferentialDrive(leftLeader, rightLeader);
    m_odometry = new DifferentialDriveOdometry(pidgey.getRotation2d());

    // Set minimum time to ramp up (originally .8)
    leftLeader.configOpenloopRamp(0.2);
   rightLeader.configOpenloopRamp(0.2);
  }

  public void setSpeed(double lSpeed, double rSpeed) {
    leftLeader.set(ControlMode.PercentOutput, lSpeed);
    rightLeader.set(ControlMode.PercentOutput, rSpeed);
    m_drive.feed();
  }

  public void stop() {
    setSpeed(0, 0);
  }

  public void setVoltage(double lVoltage, double rVoltage) {
    leftLeader.setVoltage(lVoltage);
    rightLeader.setVoltage(rVoltage);
    System.out.println("L: " + lVoltage + " R: " + rVoltage);
    m_drive.feed();
  }

  public void resetEncoder() {
    leftLeader.getSensorCollection().setIntegratedSensorPosition(0, 30);
    rightLeader.getSensorCollection().setIntegratedSensorPosition(0, 30);
    // try {
    //   Thread.sleep(50);
    // } catch (Exception e) {
    //   DriverStation.reportError("Reset Encoders", e.getStackTrace());
    // }
  }

  public void zeroAllSensors() {
    resetEncoder();
    pidgey.reset();
    pidgey.setYaw(0, 30);
    // pidgey.setAccumZAngle(0, 30);
  }

  public void feedDrive() {
    m_drive.feed();
  }

  public void resetOdometry(Pose2d pose) {
    zeroAllSensors();
    m_odometry.resetPosition(pose, pidgey.getRotation2d()); //odometry angles have left as positive
  }

  /**
   * True is coast mode, and False is brake mode
   * @param isMotorControl
   */
  public void breakVcoast(boolean isMotorControl){
    if(isMotorControl){
      leftLeader.setNeutralMode(NeutralMode.Coast);
      rightLeader.setNeutralMode(NeutralMode.Coast);
      }else{
        leftLeader.setNeutralMode(NeutralMode.Brake);
        rightLeader.setNeutralMode(NeutralMode.Brake);
      }
    }

  

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  // public double getMotorSpeed(TalonFX motor) {
  //   SmartDashboard.putNumber("LeftMotorSpeed", ((leftLeader.getSelectedSensorVelocity() / Constants.UNITS_PER_REVOLUTION * 10) / Constants.WHEEL_GEAR_RATIO)* Constants.WHEEL_CIRC_METERS);
  //   SmartDashboard.putNumber("RightMotorSpeed", ((rightLeader.getSelectedSensorVelocity() / Constants.UNITS_PER_REVOLUTION * 10) / Constants.WHEEL_GEAR_RATIO)* Constants.WHEEL_CIRC_METERS);
  //   return ((motor.getSelectedSensorVelocity() / Constants.UNITS_PER_REVOLUTION * 10) / Constants.WHEEL_GEAR_RATIO)* Constants.WHEEL_CIRC_METERS;
  // }

  public double getMotorSpeed(TalonFX motor) {
    // return motor.getInverted() == true ? ((rightLeader.getSelectedSensorVelocity() / Constants.UNITS_PER_REVOLUTION * -10) / Constants.WHEEL_GEAR_RATIO)* Constants.WHEEL_CIRC_METERS : ((rightLeader.getSelectedSensorVelocity() / Constants.UNITS_PER_REVOLUTION * 10) / Constants.WHEEL_GEAR_RATIO)* Constants.WHEEL_CIRC_METERS;
    return ((motor.getSelectedSensorVelocity() / Constants.UNITS_PER_REVOLUTION * 10) / Constants.WHEEL_GEAR_RATIO)* Constants.WHEEL_CIRC_METERS;
  }

  public double getMotorPosition(TalonFX motor) {
    return Utility.encodersToMeters(motor.getSelectedSensorPosition());
  }
/**
 *
 * @param motor
 * 0 is left motor, 1 is right motor
 * @return Meters
 */
  public double getMotorPositionExt(int motor) {
    return motor == 0 ? Utility.encodersToMeters(leftLeader.getSelectedSensorPosition()) : Utility.encodersToMeters(rightLeader.getSelectedSensorPosition());
  }

  public double getRawMotorPosition(int motor) {
    return motor == 0 ? leftLeader.getSelectedSensorPosition() : rightLeader.getSelectedSensorPosition();
  }

  public double getPidgeonYaw() {
    // return pidgey.getYaw();
    SmartDashboard.putNumber("autoYaw", pidgey.getAngle());
    return pidgey.getAngle();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    SmartDashboard.putNumber("leftMotorSpeed", getMotorSpeed(leftLeader));
    SmartDashboard.putNumber("rightMotorSpeed", getMotorSpeed(rightLeader));
    return new DifferentialDriveWheelSpeeds(getMotorSpeed(leftLeader), getMotorSpeed(rightLeader));
  }

  @Override
  public void periodic() {
  //uses .fromDegrees because Rotation2d by default uses radians. that was probably one of the big problems for pathweaver
  Rotation2d yawRot = pidgey.getRotation2d(); //odometry angles have left as positive
   m_odometry.update(yawRot, getMotorPosition(leftLeader), getMotorPosition(rightLeader));

   SmartDashboard.putNumber("autoLeftMotor", getMotorPosition(leftLeader));
   SmartDashboard.putNumber("autoRightMotor", getMotorPosition(rightLeader));
   SmartDashboard.putNumber("odoXM", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("odoYM", m_odometry.getPoseMeters().getY());
    // This method will be called once per scheduler run
  }
}