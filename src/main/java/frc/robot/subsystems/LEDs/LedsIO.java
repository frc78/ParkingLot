// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface LedsIO{

  public default void setMode(LedMode mode){}

  public static enum LedMode{
// will put constant checks in here
Shoot_Accuracy,
  }
}

