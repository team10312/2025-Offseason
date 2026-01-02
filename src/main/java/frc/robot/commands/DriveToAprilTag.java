package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.generated.Constants;
import frc.robot.generated.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Leds.AnimationType;

import java.util.List;

public class DriveToAprilTag extends Command {

  private final CommandSwerveDrivetrain drivetrain;
  private Command followCommand;

  private static final AprilTagFieldLayout FIELD =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  private static final Pose2d SIM_TAG_POSE =
      FIELD.getTagPose(17).orElseThrow().toPose2d();

  public DriveToAprilTag(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  @Override
  public void initialize() {
    if (followCommand != null && followCommand.isScheduled()) return;

    new AnimateLed(AnimationType.Rainbow).schedule();

    Pose2d robotPose = drivetrain.getEstimatedPose();

    Pose2d tagOdomPose;
    int tagId = 17;

    if (Utils.isSimulation()) {
      tagOdomPose = SIM_TAG_POSE;
    } else {
      if (!LimelightHelpers.getTV(Constants.limelightName)) return;

      tagId = (int) LimelightHelpers.getFiducialID(Constants.limelightName);

      Pose2d tagRobotRelative = drivetrain.getAprilTagPose();
      Transform2d robotToTag =
          new Transform2d(tagRobotRelative.getTranslation(), new Rotation2d());
      tagOdomPose = robotPose.transformBy(robotToTag);
    }

    Pose2d tagLayoutPose =
        FIELD.getTagPose(tagId).map(p -> p.toPose2d()).orElse(null);

    boolean useLayoutNormal =
        Utils.isSimulation()
            || (tagLayoutPose != null
                && tagOdomPose.getTranslation()
                    .getDistance(tagLayoutPose.getTranslation()) < 2.0);

    Translation2d targetTranslation;
    Rotation2d targetRotation;

    if (useLayoutNormal && tagLayoutPose != null) {
      Translation2d outward =
          new Translation2d(1.0, 0.0).rotateBy(tagLayoutPose.getRotation());

      Translation2d baseTranslation =
          Utils.isSimulation()
              ? tagLayoutPose.getTranslation()
              : tagOdomPose.getTranslation();

      targetTranslation =
          baseTranslation.plus(outward.times(Constants.aprilTagTolerance));

      targetRotation = tagLayoutPose.getRotation().plus(Rotation2d.k180deg);
    } else {
      Translation2d tagToRobot =
          robotPose.getTranslation().minus(tagOdomPose.getTranslation());

      double dist = tagToRobot.getNorm();

      Translation2d standoffDir =
          dist > 1e-9
              ? tagToRobot.div(dist)
              : new Translation2d(1.0, 0.0).rotateBy(robotPose.getRotation());

      targetTranslation =
          tagOdomPose.getTranslation()
              .plus(standoffDir.times(Constants.aprilTagTolerance));

      targetRotation =
          tagOdomPose.getTranslation().minus(targetTranslation).getAngle();
    }

    Pose2d targetPose = new Pose2d(targetTranslation, targetRotation);

    Logger.recordOutput("DriveToTag/UsingLayoutNormal", useLayoutNormal);
    Logger.recordOutput("DriveToTag/TagPose", tagOdomPose);
    Logger.recordOutput("DriveToTag/TargetPose", targetPose);

    SmartDashboard.putNumberArray(
        "DriveToTag/TargetPose",
        new double[] {
            targetPose.getX(),
            targetPose.getY(),
            targetPose.getRotation().getRadians()
        }
    );

    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(
                robotPose.getTranslation(),
                targetTranslation.minus(robotPose.getTranslation()).getAngle()),
            new Pose2d(
                targetTranslation,
                targetTranslation.minus(robotPose.getTranslation()).getAngle()));

    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            Constants.constraints,
            null,
            new GoalEndState(0.0, targetRotation));

    path.preventFlipping = true;

    followCommand = AutoBuilder.followPath(path);
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
    return false;
  }
}
