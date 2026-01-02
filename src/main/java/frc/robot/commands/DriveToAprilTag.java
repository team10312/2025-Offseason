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
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    if (followCommand != null && followCommand.isScheduled()) return;

    new AnimateLed(AnimationType.Rainbow).schedule();

    Pose2d robotPose = drivetrain.getEstimatedPose();
    Pose2d tagPose;

    if (Utils.isSimulation()) {
      tagPose = SIM_TAG_POSE;
    } else {
      if (!LimelightHelpers.getTV(Constants.limelightName)) return;
      tagPose = drivetrain.getAprilTagPose();
    }

    Logger.recordOutput("DriveToTag/TagPose", tagPose);
    SmartDashboard.putNumberArray(
        "DriveToTag/TagPose",
        new double[] {
            tagPose.getX(),
            tagPose.getY(),
            tagPose.getRotation().getRadians()
        }
    );

    Pose2d targetPose;

    if (Utils.isSimulation()) {
      targetPose = tagPose;
    } else {
      Pose2d tagOdomPose =
          robotPose.transformBy(
              new Transform2d(tagPose.getTranslation(), new Rotation2d()));

      Pose2d tagLayoutPose =
          FIELD.getTagPose(17).map(p -> p.toPose2d()).orElse(null);

      boolean useLayoutNormal =
          tagLayoutPose != null
              && tagOdomPose.getTranslation()
                  .getDistance(tagLayoutPose.getTranslation()) < 2.0;

      Translation2d targetTranslation;

      if (useLayoutNormal) {
        Translation2d outward =
            new Translation2d(1.0, 0.0).rotateBy(tagLayoutPose.getRotation());

        targetTranslation =
            tagOdomPose.getTranslation()
                .plus(outward.times(Constants.aprilTagTolerance));
      } else {
        Translation2d tagToRobot =
            robotPose.getTranslation().minus(tagOdomPose.getTranslation());

        Translation2d dir =
            tagToRobot.getNorm() > 1e-9
                ? tagToRobot.div(tagToRobot.getNorm())
                : new Translation2d(1.0, 0.0).rotateBy(robotPose.getRotation());

        targetTranslation =
            tagOdomPose.getTranslation()
                .plus(dir.times(Constants.aprilTagTolerance));
      }

      targetPose = new Pose2d(targetTranslation, robotPose.getRotation());
    }

    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(
                robotPose.getTranslation(),
                targetPose.getTranslation()
                    .minus(robotPose.getTranslation())
                    .getAngle()),
            new Pose2d(
                targetPose.getTranslation(),
                targetPose.getTranslation()
                    .minus(robotPose.getTranslation())
                    .getAngle()));

    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            Constants.constraints,
            null,
            new GoalEndState(
                0.0, targetPose.getRotation().plus(Rotation2d.k180deg)));

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
