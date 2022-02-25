// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.PathweaverAutos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Shoot.SpinUp;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Chassis.Chassis;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto1BallPar extends ParallelRaceGroup {
  /** Creates a new AutoTaxi. */
  public Auto1BallPar(RamseteCommand ramseteCommand1, Shooter m_shooter, Intake intake, Feed feed, Indexer indexer) {
    addCommands(
      new Auto1BallSeq(ramseteCommand1),
      new SpinUp(m_shooter, Constants.spinupVel),
      new IntakeCommand(intake, feed, indexer, true)
      );
  }
}
