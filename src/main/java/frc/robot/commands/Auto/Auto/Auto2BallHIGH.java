// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Auto.AutoStraight;
import frc.robot.commands.Auto.AutoTurn;
import frc.robot.commands.Auto.FireAuto;
import frc.robot.commands.Shoot.FireAUTO;
import frc.robot.subsystems.Feed;
import frc.robot.subsystems.FeedWheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Chassis.Chassis;
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto2BallHIGH extends SequentialCommandGroup {
  /** Creates a new Auto2BallHIGH. */
  // Chassis chassis;
  Timer timer;
  public Auto2BallHIGH(Chassis chassis, Intake intake, Indexer index, Feed feed, FeedWheel feedWheel, Shooter shooter, double distance, double distance2, double spinUpVel, double degrees, boolean isHood) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // this.chassis = chassis;
    addCommands(
      new InstantCommand(() -> chassis.breakVcoast(false), chassis),
      new DriveAndIntake(chassis, intake, index, distance, -0.3),
      new InstantCommand(() -> chassis.resetEncoder(), chassis),
      new WaitCommand(0.5),
      new AutoStraight(chassis, distance2, 0.3),
      new AutoTurnandSpinUp(chassis, shooter, spinUpVel, degrees, 0.2, true),
      new SpinAndFire(shooter, feed, index, feedWheel, intake, spinUpVel)
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
          // try {
      //   timer.wait(0); // in milliseconds
      // } catch (Exception exeption) {
      //   DriverStation.reportError(exeption.getMessage(), exeption.getStackTrace());
      // } 
}
