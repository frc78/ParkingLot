// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Chassis;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.CTRE_Functions;

public class Chassis extends SubsystemBase {

  // Drive Motors
  protected TalonFX leftLeader, rightLeader;

  // Motor Configs
  private TalonFXConfiguration leftConfig, rightConfig;

  // PigeonIMU
  private PigeonIMU pidgy;

  //
  private BufferedTrajectoryPointStream bufferedStream;

  /** Creates a new Chassis. */
  public Chassis() {

    bufferedStream = new BufferedTrajectoryPointStream();
    
    // Set the motor IDs
    leftLeader = new TalonFX(Constants.LLEADER);

    rightLeader = new TalonFX(Constants.RLEADER);

    // Set PigeonIMU ID
    pidgy = new PigeonIMU(Constants.IMU);

    //    Set Motor Config
    // Left
    // leftConfig.supplyCurrLimit.enable = true;
    // leftConfig.supplyCurrLimit.triggerThresholdCurrent = 40;
    // leftConfig.supplyCurrLimit.triggerThresholdTime = 1.5;
    // leftConfig.supplyCurrLimit.currentLimit = 30;

    // Right
    // rightConfig.supplyCurrLimit.enable = true;
    // rightConfig.supplyCurrLimit.triggerThresholdCurrent = 40;
    // rightConfig.supplyCurrLimit.triggerThresholdTime = 1.5;
    // rightConfig.supplyCurrLimit.currentLimit = 30;
    
    rightConfig.remoteFilter0.remoteSensorDeviceID = pidgy.getDeviceID();
    rightConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.Pigeon_Yaw;

    rightConfig.remoteFilter1.remoteSensorDeviceID = leftLeader.getDeviceID();
    rightConfig.remoteFilter1.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor;

    CTRE_Functions.setRobotDistanceConfigs(Constants.RIGHT_INVERT, rightConfig);

    rightConfig.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();

    rightConfig.neutralDeadband = Constants.NEUTRAL_DEADBAND;
    rightConfig.slot0.kF = Constants.GAINS_MOTOR_PRIFILE.kF;
    rightConfig.slot0.kP = Constants.GAINS_MOTOR_PRIFILE.kP;
    rightConfig.slot0.kI = Constants.GAINS_MOTOR_PRIFILE.kI;
    rightConfig.slot0.kD = Constants.GAINS_MOTOR_PRIFILE.kD;
    rightConfig.slot0.integralZone = (int) Constants.GAINS_MOTOR_PRIFILE.kIzone;
    rightConfig.slot0.closedLoopPeakOutput = Constants.GAINS_MOTOR_PRIFILE.kPeakOutput;
    rightConfig.slot1.kF = Constants.GAINS_MOTOR_PRIFILE.kF;
    rightConfig.slot1.kP = Constants.GAINS_MOTOR_PRIFILE.kP;
    rightConfig.slot1.kI = Constants.GAINS_MOTOR_PRIFILE.kI;
    rightConfig.slot1.kD = Constants.GAINS_MOTOR_PRIFILE.kD;
    rightConfig.slot1.integralZone = (int) Constants.GAINS_MOTOR_PRIFILE.kIzone;
    rightConfig.slot1.closedLoopPeakOutput = Constants.GAINS_MOTOR_PRIFILE.kPeakOutput;

    // Configure motors
    leftLeader.configAllSettings(leftConfig);

    rightLeader.configAllSettings(rightConfig);

    // Set Inversion
    leftLeader.setInverted(Constants.LEFT_INVERT);

    rightLeader.setInverted(Constants.RIGHT_INVERT);

    // Set polling rates
    rightLeader.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20);
    rightLeader.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20);
    rightLeader.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 20);

  }

  public void zeroAllSensors() {
    leftLeader.getSensorCollection().setIntegratedSensorPosition(0, 100);
    rightLeader.getSensorCollection().setIntegratedSensorPosition(0, 100);
    pidgy.setYaw(0);
  }

  public BufferedTrajectoryPointStream getTrajectoryStream() {
    return bufferedStream;
  }

  public void startMotionProfile() {
    leftLeader.follow(rightLeader, FollowerType.AuxOutput1);
    rightLeader.startMotionProfile(bufferedStream, 10, TalonFXControlMode.MotionProfileArc.toControlMode());
  }

  public boolean getMotionProfileOver() {
    return rightLeader.isMotionProfileFinished();
  }

  public void setSpeed(double lSpeed, double rSpeed) {
    leftLeader.set(ControlMode.PercentOutput, lSpeed);
    rightLeader.set(ControlMode.PercentOutput, rSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}