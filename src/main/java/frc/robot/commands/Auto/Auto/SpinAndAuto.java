// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Shoot.SpinUp;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.FeedWheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Chassis.Chassis;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpinAndAuto extends ParallelCommandGroup {
  /** Creates a new SpinAndAuto. */
  public SpinAndAuto(Chassis m_chassis, Shooter m_shooter, Intake m_intake, Feed m_feed, FeedWheel m_feedWheel, Indexer m_indexer, double dist, double dist2, double vel, double deg) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SpinUp(m_shooter, Constants.spinupVel2), new Auto2BallHIGH(m_chassis, m_intake, m_indexer, m_feed, m_feedWheel, m_shooter, dist, dist2, vel, deg));
  }
}
