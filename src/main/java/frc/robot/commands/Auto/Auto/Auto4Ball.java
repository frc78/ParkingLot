// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Auto.AutoStraight;
import frc.robot.commands.Auto.AutoTurn;
import frc.robot.commands.Intake.IntakeWaitAuto;
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
public class Auto4Ball extends SequentialCommandGroup {
  /** Creates a new Auto4Ball. */
  public Auto4Ball(Chassis chassis, Intake intake, Indexer indexer, Feed feed, FeedWheel wheely, Shooter shooter,double distance1, double distance2, double distance3, double distance4, double velocity, double deg, double deg2, double deg3, boolean isHood) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> chassis.breakVcoast(false), chassis),
      new DriveAndIntake(chassis, intake, indexer, distance1, -.6),
      new WaitCommand(1),
      new AutoStraight(chassis, distance2, 0.6),
      new AutoTurnandSpinUp(chassis, shooter, velocity, deg, .2, true),
      new SpinAndFire(shooter, feed, indexer, wheely, intake, velocity),
     // new FireAUTO(feed, wheely, indexer, intake, 2),//the 2 is subject to change, 2 is indicated as time (seconds)
      new AutoTurn(chassis, deg2, .2),
      new DriveAndIntake(chassis, intake, indexer, distance3, -.6),
      new IntakeWaitAuto(intake, feed, indexer, 3),
      new AutoTurn(chassis, deg3, .2),
      new DriveAndSpinUp(chassis, shooter, distance4, velocity, true),
      new SpinAndFire(shooter, feed, indexer, wheely, intake, velocity)
     // new FireAUTO(feed, wheely, indexer, intake, 6)
    );
  }
}
