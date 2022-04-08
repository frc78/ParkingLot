// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.PathweaverAutos;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Auto.AutoStraight;
import frc.robot.commands.Auto.AutoTurn;
import frc.robot.commands.Auto.FireAuto;
import frc.robot.commands.Shoot.FireAUTO;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.FeedWheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Chassis.Chassis;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto4BallV2Seq extends SequentialCommandGroup {
  /** Creates a new Auto4BallV2Seq. */
  public Auto4BallV2Seq(Chassis chassis, Intake intake, Feed feed, FeedWheel feedwheel, Indexer indexer, RamseteCommand path1, RamseteCommand path2, Trajectory trajectory1) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoStraight(chassis, 2, 0.2),
      new AutoTurn(chassis, 190, 0.2),
      new FireAUTO(feed, feedwheel, indexer, intake, 2),
      new AutoTurn(chassis, 190, 0.2),
      new InstantCommand(() -> chassis.resetOdometry(trajectory1.getInitialPose()), chassis),
      path1,
      new WaitCommand(1),
      // new AutoTurn(chassis, 180, 0.2),
      path2,
      new FireAUTO(feed, feedwheel, indexer, intake, 2)
    );
  }
}
