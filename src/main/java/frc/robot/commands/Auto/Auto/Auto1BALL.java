// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants;
import frc.robot.commands.Intake.Tuck;
import frc.robot.commands.Shoot.Fire;
import frc.robot.commands.Shoot.FireAUTO;
import frc.robot.commands.Shoot.SpinUp;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.FeedWheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto1BALL extends ParallelRaceGroup {
  /** Creates a new Auto1BALL. */
  private Timer timer;
  public Auto1BALL(Shooter shoot, Feed feed, Indexer index, FeedWheel feedWheel, Intake intake ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SpinUp(shoot, Constants.spinupVel2),
      new FireAUTO(feed, feedWheel, index, intake, 3)
    );
  }
  
 // @Override
   /* public void initialize() {
      try {
        timer.wait(0); // in milliseconds
      } catch (Exception exeption) {
        DriverStation.reportError(exeption.getMessage(), exeption.getStackTrace());
      } 
    }*/
}