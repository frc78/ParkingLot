// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.Auto.AutoStraight;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Intake.IntakeNoFeed;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Chassis.Chassis;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveAndIntake extends ParallelRaceGroup {
  /** Creates a new DriveAndFeed. */
  public DriveAndIntake(Chassis chassis, Intake intake, Indexer indexer, double distance, double speed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoStraight(chassis, distance, speed),
      new IntakeNoFeed(intake, indexer)
    );
  }
}
