// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallCount extends SubsystemBase {
  /** Creates a new BallCount. */
  static double balls;

  public BallCount() {}

  public static double returnBallCount() {

    return balls;
  }

  public static void increaseBallCount(double i) {
    balls= balls+i;
  }

  
  public static void resetBallCount() {
    balls= 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
