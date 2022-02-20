// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Chassis;

import java.sql.Driver;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
    leftLeader.setNeutralMode(Constants.MOTOR_MODE);

    rightLeader.setNeutralMode(Constants.MOTOR_MODE);

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
    leftLeader.configOpenloopRamp(0.78);
    rightLeader.configOpenloopRamp(0.78);
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
    DriverStation.reportWarning("L: " + lVoltage + " R: " + rVoltage, false);
    m_drive.feed();
  }

  public void resetEncoder() {
    leftLeader.getSensorCollection().setIntegratedSensorPosition(0, 30);
    rightLeader.getSensorCollection().setIntegratedSensorPosition(0, 30);
  }

  public void zeroAllSensors() {
    resetEncoder();
    pidgey.setYaw(0, 30);
    pidgey.setAccumZAngle(0, 30);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoder();
    m_odometry.resetPosition(pose, pidgey.getRotation2d());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public double getMotorSpeed(TalonFX motor) {
    return (motor.getSelectedSensorVelocity() / Constants.UNITS_PER_REVOLUTION * 10) * Constants.WHEEL_CIRC_METERS / Constants.WHEEL_GEAR_RATIO;
  }

  public double getMotorPosition(TalonFX motor) {
    return (motor.getSelectedSensorPosition() / Constants.UNITS_PER_REVOLUTION) * Constants.WHEEL_CIRC_METERS / Constants.WHEEL_GEAR_RATIO;
  }

  public double getRawMotorPosition(int motor) {
    return motor == 0 ? leftLeader.getSelectedSensorPosition() : rightLeader.getSelectedSensorPosition();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getMotorSpeed(leftLeader), getMotorSpeed(rightLeader));
  }

  @Override
  public void periodic() {
  //  m_odometry.update(pidgey.getRotation2d(), getMotorPosition(leftLeader), getMotorPosition(rightLeader));

    SmartDashboard.putNumber("left encoder", leftLeader.getSelectedSensorVelocity());
    SmartDashboard.putNumber("right encoder", rightLeader.getSelectedSensorVelocity());
    // This method will be called once per scheduler run
  }
}