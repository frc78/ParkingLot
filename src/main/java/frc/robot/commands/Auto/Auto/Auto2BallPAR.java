// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Auto.AutoTurn;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Shoot.Fire;
import frc.robot.commands.Shoot.SpinUp;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.FeedWheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Chassis.Chassis;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto2BallPAR extends ParallelCommandGroup {
  /** Creates a new Auto2BallPAR. */
  public Auto2BallPAR(Chassis chassis, IntakeCommand intake, Shooter shoot, Indexer indexer, Feed feed, FeedWheel feedWheel) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SpinUp(shoot),
      new AutoTurn(chassis, 180, .2),
      new Fire(feed, indexer, feedWheel)


    );
  }
}
