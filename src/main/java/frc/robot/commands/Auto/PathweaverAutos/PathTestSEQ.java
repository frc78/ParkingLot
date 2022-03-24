// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.PathweaverAutos;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Chassis.Chassis;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathTestSEQ extends SequentialCommandGroup {
  /** Creates a new PathTestSEQ. */
  Chassis m_chassis;
  public PathTestSEQ(RamseteCommand ramseteCommand1, RamseteCommand ramseteCommand2, Chassis chassis) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_chassis = chassis;
    addRequirements(m_chassis);
    addCommands(
      // m_chassis.resetOdometry(trajectory1.getInitialPose()),
      ramseteCommand1,
      ramseteCommand2
    );
  }
}
