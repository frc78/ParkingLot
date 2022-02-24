// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Auto.AutoStraight;
import frc.robot.commands.Auto.AutoTurn;
import frc.robot.commands.Auto.FireAuto;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.FeedWheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Chassis.Chassis;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto1Ball3Seq extends SequentialCommandGroup {
  /** Creates a new AutoTaxi1. */
  public Auto1Ball3Seq(Chassis m_chassis, Indexer m_indexer, Feed m_feed, FeedWheel m_feedWheel) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new FireAuto(m_indexer, m_feed, m_feedWheel),
      new AutoStraight(m_chassis, 2.1, 0.25),
      new AutoTurn(m_chassis, -155, 0.15),
      new AutoStraight(m_chassis, 2.1, 0.25),
      new AutoTurn(m_chassis, -20, 0.15),
      new FireAuto(m_indexer, m_feed, m_feedWheel)
    );
  }
}
