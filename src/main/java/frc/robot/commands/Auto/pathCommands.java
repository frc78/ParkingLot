package frc.robot.commands.Auto;
import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Chassis.Chassis;
import frc.robot.Constants;

public class pathCommands {

public RamseteCommand createRamseteCommand(Trajectory trajectory, Chassis m_chassis) {
        return new RamseteCommand(
            trajectory, 
            m_chassis::getPose, 
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta), 
            new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter), 
            Constants.kDriveKinematics, 
            m_chassis::getWheelSpeeds, 
            new PIDController(Constants.kPDriveVel, 0, 0), 
            new PIDController(Constants.kPDriveVel, 0, 0), 
            m_chassis::setVoltage, 
            m_chassis
          );
    }

  public Trajectory createTrajectory(String json) {
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(json);
      return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      //DriverStation.reportError("Unable to open trajectory: " + json, ex.getStackTrace());
      return null;
    }
  }
}
