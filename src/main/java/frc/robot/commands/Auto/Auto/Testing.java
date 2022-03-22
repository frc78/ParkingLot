// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Auto;

import com.fasterxml.jackson.databind.jsontype.impl.AsDeductionTypeDeserializer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Auto.AutoStraight;
import frc.robot.commands.Auto.AutoTurn;
import frc.robot.subsystems.Chassis.Chassis;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Testing extends SequentialCommandGroup {
  /** Creates a new Testing. */
  public Testing(Chassis m_chassis) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // addCommands(
    //   new AutoStraight(m_chassis, 0.7, 0.2),
    //   new AutoTurn(m_chassis, 40, 0.15),
    //   new AutoStraight(m_chassis, 3, 0.2),
    //   new AutoTurn(m_chassis, 70, 0.2),
    //   new AutoStraight(m_chassis, 3, 0.2)
    // );
    addCommands(
      new InstantCommand(() -> m_chassis.breakVcoast(true), m_chassis),
      new AutoStraight(m_chassis, 2, 0.4),
      new AutoTurn(m_chassis, 175, 0.2),
      new AutoStraight(m_chassis, 2, 0.4),
      new AutoTurn(m_chassis, -175, 0.2)
       );
  }
}
