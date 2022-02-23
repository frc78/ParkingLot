// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.BackupAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Auto.AutoStraight;
import frc.robot.commands.Auto.AutoTurn;
import frc.robot.subsystems.Chassis.Chassis;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BackupAutoTestSeq extends SequentialCommandGroup {
  /** Creates a new BackupAuto1Seq. */
  public BackupAutoTestSeq(Chassis subsystem1) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoStraight(subsystem1, 1, 0.1),
      new AutoTurn(subsystem1, 180, 0.15),
      new AutoStraight(subsystem1, 1, 0.1),
      new AutoTurn(subsystem1, 180, 0.15)
    );
  }
}
