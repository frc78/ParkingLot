// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.PathweaverAutos;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Chassis.Chassis;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto4BallPathSeq extends SequentialCommandGroup {
  /** Creates a new Auto4BallPathSeq. */
  public Auto4BallPathSeq(RamseteCommand rCommand1, RamseteCommand rCommand2, RamseteCommand rCommand3, RamseteCommand rCommand4, RamseteCommand rCommand5, Chassis chassis) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(chassis);
    addCommands(
      rCommand1,
      rCommand2,
      rCommand3,
      rCommand4,
      rCommand5
      );
  }
}
