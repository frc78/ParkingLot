// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoTaxi extends SequentialCommandGroup {
  /** Creates a new AutoTaxi. */
  public AutoTaxi(RamseteCommand path1, RamseteCommand path2) {
  super(path1, /*put in the auto-shooter comand*/ path2); 
  }
}