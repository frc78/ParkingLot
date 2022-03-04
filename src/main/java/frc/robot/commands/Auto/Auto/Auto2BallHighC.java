// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Auto.AutoStraight;
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
public class Auto2BallHighC extends SequentialCommandGroup {
  /** Creates a new Auto2BallC. */
  public Auto2BallHighC(Chassis chassis, Intake intake, Indexer indexer, Shooter shooter, Feed feed, FeedWheel feedWheel) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> chassis.breakVcoast(false), chassis),
      new DriveAndIntake(chassis, intake, indexer, 1.5, 0.3),
      new AutoStraight(chassis, 1.5, -0.3),
      new AutoTurnandSpinUp(chassis, shooter, Constants.spinupVel2, 175, 0.2),
      new FireAUTO(feed, feedWheel, indexer, intake, 3)
    );
  }
}
