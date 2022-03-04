// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Auto.AutoStraight;
import frc.robot.commands.Shoot.Fire;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.FeedWheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Chassis.Chassis;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AUTO1BALLSEQ extends SequentialCommandGroup {
  /** Creates a new AUTO1BALLSEQ. */
  // Chassis chassis;
  public AUTO1BALLSEQ(Chassis chassis, Feed feed, Indexer index, Shooter shooter, FeedWheel feedWheel, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // this.chassis = chassis;
    addCommands(
      new InstantCommand(() -> chassis.breakVcoast(false), chassis),
      new Auto1BALL(shooter, feed, index, feedWheel, intake),
      new AutoStraight(chassis, 2, -.3)
    );
  }
  // @Override
  // public void initialize() {
  //   chassis.breakVcoast(false);
  // }
  // @Override
  // public void end(boolean wasInterrupted) {
  //   chassis.breakVcoast(Constants.MOTOR_MODE == NeutralMode.Coast ? true : false);
  // }
}
