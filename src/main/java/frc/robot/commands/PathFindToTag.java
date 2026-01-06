package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.generated.Constants;
import frc.robot.generated.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Leds.AnimationType;

import java.util.List;

public class PathFindToTag extends Command {

  private final CommandSwerveDrivetrain drivetrain;
  private Command followCommand;
  private boolean isFinished = false;
  private static final AprilTagFieldLayout FIELD =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  private static final Pose2d SIM_TAG_POSE =
      FIELD.getTagPose(17).orElseThrow().toPose2d();

  public PathFindToTag(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  @Override
  public void initialize() {
    if (followCommand != null && followCommand.isScheduled()) return;

    new AnimateLed(AnimationType.Rainbow).schedule();
    
    drivetrain.resetPose(drivetrain.getFieldRelativeRobotPose());

    Pose2d targetPose = new Pose2d();

    if (Utils.isSimulation()) {
      targetPose = SIM_TAG_POSE;
    }
    else if (LimelightHelpers.getTV(Constants.limelightName)){
      targetPose = drivetrain.getAprilTagFieldRelativePose();
    }
    else{
      return;
    }

    Translation2d outward =
      new Translation2d(1.0, 0.0).rotateBy(targetPose.getRotation());

    Translation2d targetTranslation =
        targetPose.getTranslation().plus(outward.times(Constants.aprilTagTolerance));

    Rotation2d targetRotation = targetPose.getRotation().plus(Rotation2d.k180deg);

    targetPose = new Pose2d(targetTranslation, targetRotation);

    Logger.recordOutput("DriveToTag/TargetPose", targetPose);

    SmartDashboard.putNumberArray(
        "DriveToTag/TargetPose",
        new double[] {
            targetPose.getX(),
            targetPose.getY(),
            targetPose.getRotation().getRadians()
        }
    );

    PathPlannerLogging.logTargetPose(targetPose);

    followCommand = AutoBuilder.pathfindToPose(targetPose, Constants.constraints);
    followCommand.schedule();
  }

  @Override
  public void end(boolean interrupted) {
    if (followCommand != null) {
      followCommand.cancel();
      followCommand = null;
    }

    drivetrain.stop();

    if (interrupted) {
      new DefaultLed().schedule();
    } else {
      new SetLedColor(0, 255, 0).schedule();
    }
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
