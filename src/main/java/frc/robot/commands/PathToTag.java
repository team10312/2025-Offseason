package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.generated.Constants;
import frc.robot.generated.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Leds.AnimationType;

import java.util.List;

public class PathToTag extends Command {

  private final CommandSwerveDrivetrain drivetrain;
  private Command followCommand;

  private static final Pose2d SIM_TAG_POSE =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded)
          .getTagPose(17)
          .orElseThrow()
          .toPose2d();

  public PathToTag(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    // IMPORTANT: prevent rescheduling while held
    if (followCommand != null && followCommand.isScheduled()) return;

    new AnimateLed(AnimationType.Rainbow).schedule();

    Pose2d robotPose = drivetrain.getEstimatedPose();
    Pose2d tagPose;

    if (Utils.isSimulation()) {
      tagPose = SIM_TAG_POSE;
    } else {
      if (!LimelightHelpers.getTV(Constants.limelightName)) return;
      tagPose = drivetrain.getAprilTagPose(); // robot-relative
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

    Pose2d targetPose = Utils.isSimulation()
        ? tagPose
        : new Pose2d(
            robotPose.getX() + tagPose.getX(),
            robotPose.getY() - tagPose.getY(),
            robotPose.getRotation()
        );

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(
            robotPose.getTranslation(),
            targetPose.getTranslation().minus(robotPose.getTranslation()).getAngle()
        ),
        new Pose2d(
            targetPose.getTranslation(),
            targetPose.getTranslation().minus(robotPose.getTranslation()).getAngle()
        )
    );

    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        Constants.constraints,
        null,
        // ✅ ONLY place +180 belongs
        new GoalEndState(0.0, targetPose.getRotation().plus(Rotation2d.k180deg))
    );

    path.preventFlipping = true;

    followCommand = AutoBuilder.followPath(path);
    followCommand.schedule();
  }

  @Override
  public void end(boolean interrupted) {
    // ✅ GUARANTEED cancellation on button release
    if (followCommand != null) {
      followCommand.cancel();
      followCommand = null;
    }

    drivetrain.setControl(
        new SwerveRequest.FieldCentric()
            .withVelocityX(0.0)
            .withVelocityY(-0.0)
            .withRotationalRate(0.0)
    );

    if (interrupted) {
      new DefaultLed().schedule();
    } else {
      new SetLedColor(0, 255, 0).schedule();
    }
  }

  @Override
  public boolean isFinished() {
    return false; // required for whileTrue
  }
}
