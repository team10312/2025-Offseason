package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.generated.Constants;
import frc.robot.generated.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Leds.AnimationType;

public class PathFindToTag extends Command {

  private static final AprilTagFieldLayout FIELD =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  private final CommandSwerveDrivetrain drivetrain;
  private Command followCommand;

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
    Pose2d robotPose = drivetrain.getEstimatedPose();

    Pose2d tagOdomPose = null;
    int tagId = 17;

    if (Utils.isSimulation()) {
      tagOdomPose = SIM_TAG_POSE;
    } else {
      if (!LimelightHelpers.getTV(Constants.limelightName)) return;

      tagId = (int) LimelightHelpers.getFiducialID(Constants.limelightName);

      Pose2d tagRobotRelative = drivetrain.getAprilTagFieldRelativePose();
      Transform2d robotToTag = new Transform2d(tagRobotRelative.getTranslation(), new Rotation2d());
      tagOdomPose = robotPose.transformBy(robotToTag);
    }

    Pose2d tagLayoutPose = FIELD.getTagPose(tagId).map(p -> p.toPose2d()).orElse(null);

    boolean useLayoutNormal =
        Utils.isSimulation()
            || (tagLayoutPose != null
                && tagOdomPose != null
                && tagOdomPose.getTranslation().getDistance(tagLayoutPose.getTranslation()) < 2.0);

    Translation2d targetTranslation;
    Rotation2d targetRotation;

    if (useLayoutNormal && tagLayoutPose != null) {
      Rotation2d tagRotation = tagLayoutPose.getRotation();
      Translation2d outward = new Translation2d(1.0, 0.0).rotateBy(tagRotation);

      Translation2d tagTranslation = Utils.isSimulation()
          ? tagLayoutPose.getTranslation()
          : tagOdomPose.getTranslation();

      targetTranslation = tagTranslation.plus(outward.times(Constants.aprilTagTolerance));
      targetRotation = tagRotation.plus(Rotation2d.fromDegrees(180.0));
    } else {
      Translation2d tagToRobot = robotPose.getTranslation().minus(tagOdomPose.getTranslation());
      double dist = tagToRobot.getNorm();

      Translation2d standoffDir =
          dist > 1e-9 ? tagToRobot.div(dist)
                      : new Translation2d(1.0, 0.0).rotateBy(robotPose.getRotation());

      targetTranslation = tagOdomPose.getTranslation().plus(standoffDir.times(Constants.aprilTagTolerance));
      targetRotation = tagOdomPose.getTranslation().minus(targetTranslation).getAngle();
    }

    Pose2d targetPose = new Pose2d(targetTranslation, targetRotation);

    Logger.recordOutput("DriveToTag/UsingLayoutNormal", useLayoutNormal);
    Logger.recordOutput("DriveToTag/TagPose", tagOdomPose);
    Logger.recordOutput("DriveToTag/TargetPose", targetPose);

    SmartDashboard.putNumberArray(
        "DriveToTag/TargetPose",
        new double[] { targetPose.getX(), targetPose.getY(), targetPose.getRotation().getRadians() }
    );

    followCommand = AutoBuilder.pathfindToPose(targetPose, Constants.constraints, 0.0);
    followCommand.schedule();
  }

  @Override
  public void end(boolean interrupted) {
    if (followCommand != null) {
      followCommand.cancel();
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
