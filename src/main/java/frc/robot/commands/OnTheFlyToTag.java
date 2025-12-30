package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Leds.AnimationType;

public class OnTheFlyToTag extends Command {
  private final CommandSwerveDrivetrain drivetrain;

  private static final PathConstraints kConstraints = new PathConstraints(
      2.0,
      3.0,
      Math.toRadians(360),
      Math.toRadians(540)
  );

  private static final double kStandoffDistance = 0.5;

  private boolean inSimulation = false;

  private Pose2d targetPose = null;
  private Rotation2d targetRotation = null;

  private PathPlannerPath generatedPath = null;
  private List<PathPoint> allPathPoints = null;

  private Command followCmd = null;
  private boolean hadValidPath = false;

  private final Pose2d kSimTagFieldPose;

  private final SwerveRequest.FieldCentric stopRequest =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public OnTheFlyToTag(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    kSimTagFieldPose = drivetrain.getAprilTagFieldRelativePose();
  }

  @Override
  public void initialize() {
    new AnimateLed(AnimationType.Rainbow).schedule();
    drivetrain.setDriveToTagRunning(true);
    inSimulation = Utils.isSimulation();

    SmartDashboard.putNumberArray("OnTheFly/PathPoints", new double[0]);
    SmartDashboard.putNumberArray("OnTheFly/TargetPose", new double[0]);
    SmartDashboard.putNumberArray("OnTheFly/RobotPose", new double[0]);
    SmartDashboard.putNumberArray("OnTheFly/TagPose", new double[0]);

    generatedPath = null;
    allPathPoints = null;
    targetPose = null;
    targetRotation = null;
    followCmd = null;
    hadValidPath = false;

    // ------------------------------------------------------------
    // STEP 0: ROBOT FIELD POSE (USE YOUR DRIVETRAIN FUNCTION)
    // ------------------------------------------------------------
    Pose2d startPose = drivetrain.getRobotFieldRelativePose();
    SmartDashboard.putNumberArray(
        "OnTheFly/RobotPose",
        new double[] { startPose.getX(), startPose.getY(), startPose.getRotation().getRadians() }
    );

    // ------------------------------------------------------------
    // STEP 1: TAG FIELD POSE (USE YOUR DRIVETRAIN FUNCTION)
    // ------------------------------------------------------------
    Pose2d tagFieldPose;
    if (inSimulation) {
      tagFieldPose = kSimTagFieldPose;
    } else {
      tagFieldPose = drivetrain.getAprilTagFieldRelativePose();

      // If tag not valid (your function returns new Pose2d() on failure), bail.
      boolean tagInvalid =
          Math.abs(tagFieldPose.getX()) < 1e-6 &&
          Math.abs(tagFieldPose.getY()) < 1e-6 &&
          Math.abs(tagFieldPose.getRotation().getRadians()) < 1e-6;

      if (tagInvalid) {
        System.out.println("OnTheFly: No valid tag field pose (fiducialID <= 0 / no tag).");
        return;
      }
    }

    SmartDashboard.putNumberArray(
        "OnTheFly/TagPose",
        new double[] { tagFieldPose.getX(), tagFieldPose.getY(), tagFieldPose.getRotation().getRadians() }
    );

    Translation2d tagTranslation = tagFieldPose.getTranslation();
    Rotation2d tagRotation = tagFieldPose.getRotation();

    // ------------------------------------------------------------
    // STEP 2: TARGET POSE
    // ------------------------------------------------------------
    Translation2d approachVector =
        new Translation2d(kStandoffDistance, 0).rotateBy(tagRotation);

    Translation2d targetTranslation = tagTranslation.plus(approachVector);

    // Robot should face the tag
    targetRotation = tagRotation.plus(Rotation2d.k180deg);
    targetPose = new Pose2d(targetTranslation, targetRotation);

    SmartDashboard.putNumberArray(
        "OnTheFly/TargetPose",
        new double[] {
            targetPose.getX(),
            targetPose.getY(),
            targetPose.getRotation().getRadians()
        });

    // ------------------------------------------------------------
    // STEP 3: PATH GENERATION
    // ------------------------------------------------------------
    Rotation2d travelDir =
        targetPose.getTranslation()
            .minus(startPose.getTranslation())
            .getAngle();

    Translation2d midpoint =
        startPose.getTranslation()
            .plus(targetPose.getTranslation())
            .div(2.0);

    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(startPose.getTranslation(), travelDir),
            new Pose2d(midpoint, travelDir),
            new Pose2d(targetPose.getTranslation(), travelDir)
        );

    List<RotationTarget> holonomicRotations = List.of(
        new RotationTarget(0.0, startPose.getRotation()),
        new RotationTarget(1.0, targetRotation)
    );

    generatedPath = new PathPlannerPath(
        waypoints,
        holonomicRotations,
        new ArrayList<PointTowardsZone>(),
        new ArrayList<ConstraintsZone>(),
        new ArrayList<EventMarker>(),
        kConstraints,
        null,
        new GoalEndState(0.0, targetRotation),
        false
    );

    generatedPath.preventFlipping = true;

    allPathPoints = generatedPath.getAllPathPoints();
    if (allPathPoints == null || allPathPoints.isEmpty()) {
      return;
    }

    hadValidPath = true;
    logPathPoints();

    followCmd = AutoBuilder.followPath(generatedPath);
    followCmd.schedule();
  }

  @Override
  public void execute() {
    // No early-finish logic for testing
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setDriveToTagRunning(false);

    if (followCmd != null &&
        CommandScheduler.getInstance().isScheduled(followCmd)) {
      followCmd.cancel();
    }

    drivetrain.setControl(
        stopRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0)
    );

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

  private void logPathPoints() {
    double[] pathArray = new double[allPathPoints.size() * 2];
    for (int i = 0; i < allPathPoints.size(); i++) {
      PathPoint pt = allPathPoints.get(i);
      pathArray[i * 2] = pt.position.getX();
      pathArray[i * 2 + 1] = pt.position.getY();
    }
    SmartDashboard.putNumberArray("OnTheFly/PathPoints", pathArray);
  }
}
