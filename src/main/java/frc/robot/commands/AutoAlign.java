// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.generated.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;

/** An example command that uses an example subsystem. */
public class AutoAlign extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final CommandSwerveDrivetrain drivetrain;
    private final Pose2d targetPose;
    private final PathConstraints constraints;
    private Command pathFollowingCommand;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoAlign(CommandSwerveDrivetrain drivetrain, Pose2d targetPose, PathConstraints constraints) {
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;
        this.constraints = constraints;

        // addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetPose(drivetrain.getLLPose());
    pathFollowingCommand = AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
    // pathFollowingCommand.schedule();
    pathFollowingCommand.alongWith(PathfindingCommand.warmupCommand());
    drivetrain.pathScheduled = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    drivetrain.pathScheduled = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(drivetrain.getAprilTagPose().getX() - drivetrain.getLLPose().getX()) <= 1;
    // return false;
  }
}