// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto2BallHIGH extends SequentialCommandGroup {
  /** Creates a new Auto2BallHIGH. */
  public Auto2BallHIGH(Chassis chassis, Intake intake, Indexer index, Feed feed, FeedWheel feedWheel, Shooter shooter, double distance, double distance2, double spinUpVel, double degrees ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveAndIntake(chassis, intake, index, distance, 0.3),
      new AutoStraight(chassis, distance2, -0.3),
      new AutoTurnandSpinUp(chassis, shooter, spinUpVel, degrees, 0.2),
      new FireAUTO(feed, feedWheel, index, intake)
    );
  }
}
