// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  private final boolean replayMode = false;

  public Robot() {
    // ---------------- AdvantageKit Logging Configuration ----------------
    Logger.recordMetadata("ProjectName", "MyProject");

    if (isReal()) {
      // REAL ROBOT
      Logger.addDataReceiver(new WPILOGWriter());   // USB (/U/logs)
      Logger.addDataReceiver(new NT4Publisher());   // Live NT4
    } else if (replayMode) {
      // REPLAY SIMULATION
      setUseTiming(false);
      String logPath = LogFileUtil.findReplayLog();
      Logger.setReplaySource(new WPILOGReader(logPath));
      Logger.addDataReceiver(
          new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    } else {
      // LIVE DESKTOP SIMULATION (what you want)
      Logger.addDataReceiver(new NT4Publisher());
    }

    Logger.start();
    // -------------------------------------------------------------------

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathfindingCommand.warmupCommand().schedule();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
  }

  @Override public void disabledInit() {}
  @Override public void disabledPeriodic() {}
  @Override public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override public void autonomousPeriodic() {}
  @Override public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override public void teleopPeriodic() {}
  @Override public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override public void testPeriodic() {}
  @Override public void testExit() {}

  @Override
  public void simulationPeriodic() {
    // Physics simulation goes here (swerve, drivetrain, gyro, etc.)
  }
}
