// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Shoot.ShootAuto;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.FeedWheel;
import frc.robot.subsystems.Indexer;
import frc.robot.RobotContainer;

public class AutoTaxi extends SequentialCommandGroup {
  /** Creates a new AutoTaxi. */
  public AutoTaxi(Feed m_feed, Indexer m_indexer, FeedWheel m_feedWheel, RamseteCommand path1, RamseteCommand path2) {
    super(path1, new ShootAuto(m_feed, m_indexer, m_feedWheel), path2);
  }
}
