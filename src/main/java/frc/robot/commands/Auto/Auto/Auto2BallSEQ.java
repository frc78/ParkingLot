// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Auto.AutoTurn;
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
public class Auto2BallSEQ extends SequentialCommandGroup {
  /** Creates a new Auto2BallSEQ. */
  Chassis chassis;
  // private Timer timer;
  public Auto2BallSEQ(Chassis chassis, Intake intake, Feed feed, Shooter shooter, FeedWheel wheely, Indexer index) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // this.chassis = chassis;
    addCommands(
    new InstantCommand(() -> chassis.breakVcoast(false), chassis),
    new DriveAndIntake(chassis, intake, index, 1.5, 0.3),
    new AutoTurn(chassis, 170, .2),
    new DriveAndSpinUp(chassis, shooter, 2.4, Constants.spinupVel, true),
    new SpinAndFire(shooter, feed, index, wheely, intake, Constants.spinupVel, Constants.backSpinVel, true)
    );
  }
  // @Override
  //   public void initialize() {
  //     chassis.breakVcoast(false);
  //     try {timer.wait(0); // in milliseconds
  //     } catch (Exception exeption) {
  //       DriverStation.reportError(exeption.getMessage(), exeption.getStackTrace());
  //     } 
  //   }
  //   @Override
  //   public void end(boolean wasInterrupted) {
  //     chassis.breakVcoast(true);
  //   }
}