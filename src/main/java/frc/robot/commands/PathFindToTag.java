package frc.robot.commands;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.generated.Constants;
import frc.robot.generated.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Leds.AnimationType;

/**
 * PathFindToTag - Uses PathPlanner's AD* pathfinding algorithm to navigate
 * to an AprilTag's position using field-relative coordinates.
 * 
 * This command:
 * 1. Gets the AprilTag's field position from Limelight
 * 2. Uses PathPlanner's pathfinder to calculate an optimal path
 * 3. Follows the path using PPHolonomicDriveController (handles both translation AND rotation)
 * 4. Stops when within tolerance of the target pose
 */
public class PathFindToTag extends Command {

    private final CommandSwerveDrivetrain drivetrain;

    // Path following controller - handles BOTH translation AND rotation
    private final PPHolonomicDriveController controller;
    
    // Robot config for trajectory generation
    private final RobotConfig robotConfig;

    // Path constraints - velocity and acceleration limits
    private static final PathConstraints kConstraints = new PathConstraints(
        1.5,  // max velocity (m/s)
        2.0,  // max acceleration (m/s²)
        Math.toRadians(360),  // max angular velocity (rad/s)
        Math.toRadians(540)   // max angular acceleration (rad/s²)
    );

    // Standoff distance - how far in front of the tag to stop
    private static final double kStandoffDistance = 0.5;  // meters

    // Finish tolerances
    private static final double kPositionTolerance = 0.1;  // meters
    private static final double kRotationTolerance = Math.toRadians(5);  // radians

    // Current path state
    private PathPlannerPath currentPath = null;
    private PathPlannerTrajectory currentTrajectory = null;
    private final Timer pathTimer = new Timer();

    // Target pose (field-relative)
    private Pose2d targetPose = null;
    private GoalEndState goalEndState = null;

    // Track if we have a valid target
    private boolean hasValidTarget = false;

    public PathFindToTag(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        
        // Create controller with same PID gains as AutoBuilder config
        this.controller = new PPHolonomicDriveController(
            new PIDConstants(Constants.TRANS_KP, Constants.TRANS_KI, Constants.TRANS_KD),
            new PIDConstants(Constants.ROT_KP, Constants.ROT_KI, Constants.ROT_KD)
        );
        
        // Load robot config from PathPlanner GUI settings
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Fallback: this should not happen if PathPlanner is configured
            throw new RuntimeException("PathPlanner RobotConfig not found!", e);
        }
        this.robotConfig = config;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // 1. Turn on rainbow LEDs
        new AnimateLed(AnimationType.Rainbow).schedule();

        // 2. Reset state
        currentPath = null;
        currentTrajectory = null;
        targetPose = null;
        hasValidTarget = false;
        pathTimer.stop();
        pathTimer.reset();

        // 3. Check for visible tag
        if (!LimelightHelpers.getTV(Constants.limelightName)) {
            SmartDashboard.putString("PathFindToTag/Status", "No AprilTag visible");
            return;
        }

        // 4. Get robot's field pose using HYBRID approach:
        //    - Position (X, Y) from Limelight (accurate when tag visible)
        //    - Rotation from gyro/odometry (stable and reliable)
        Pose2d limelightPose = drivetrain.getLLPose();
        Pose2d odometryPose = drivetrain.getEstimatedPose();
        
        // Validate Limelight pose (check for invalid 0,0 readings)
        if (limelightPose.getX() == 0 && limelightPose.getY() == 0) {
            SmartDashboard.putString("PathFindToTag/Status", "Invalid robot pose from Limelight");
            return;
        }
        
        // Hybrid pose: Limelight position + Gyro rotation
        Pose2d robotFieldPose = new Pose2d(
            limelightPose.getTranslation(),  // Position from Limelight (accurate)
            odometryPose.getRotation()       // Rotation from gyro (stable)
        );
        
        SmartDashboard.putNumber("PathFindToTag/LLPoseX", limelightPose.getX());
        SmartDashboard.putNumber("PathFindToTag/LLPoseY", limelightPose.getY());
        SmartDashboard.putNumber("PathFindToTag/GyroRotDeg", odometryPose.getRotation().getDegrees());

        // 5. Get tag's robot-relative pose
        Pose2d tagRelative = drivetrain.getAprilTagPose();

        // 6. Transform tag position to field coordinates
        // tagRelative is where the tag is relative to the robot
        // We need to find where that is on the field
        Transform2d robotToTag = new Transform2d(
            tagRelative.getTranslation(),
            tagRelative.getRotation()
        );
        Pose2d tagFieldPose = robotFieldPose.transformBy(robotToTag);

        // 7. Calculate target pose - position in front of tag, facing it
        // The tag's rotation points away from its surface, so we want to be
        // in front of it (offset along the tag's facing direction) and face it
        Translation2d tagFacing = new Translation2d(1, 0).rotateBy(tagFieldPose.getRotation());
        Translation2d standoffOffset = tagFacing.times(-kStandoffDistance);  // Negative = in front
        
        targetPose = new Pose2d(
            tagFieldPose.getTranslation().plus(standoffOffset),
            tagFieldPose.getRotation().plus(Rotation2d.k180deg)  // Face the tag (180° from tag's facing)
        );

        // 8. Set up pathfinding goal
        // GoalEndState includes: end velocity (0 = stop) and target rotation
        goalEndState = new GoalEndState(0.0, targetPose.getRotation());

        // 9. Tell pathfinder where we are and where we want to go
        Pathfinding.setStartPosition(robotFieldPose.getTranslation());
        Pathfinding.setGoalPosition(targetPose.getTranslation());

        hasValidTarget = true;

        // Debug output
        SmartDashboard.putString("PathFindToTag/Status", "Pathfinding started");
        SmartDashboard.putNumber("PathFindToTag/TargetX", targetPose.getX());
        SmartDashboard.putNumber("PathFindToTag/TargetY", targetPose.getY());
        SmartDashboard.putNumber("PathFindToTag/TargetRotDeg", targetPose.getRotation().getDegrees());
        SmartDashboard.putNumber("PathFindToTag/TagFieldX", tagFieldPose.getX());
        SmartDashboard.putNumber("PathFindToTag/TagFieldY", tagFieldPose.getY());
    }

    @Override
    public void execute() {
        if (!hasValidTarget) {
            // No valid target - just stop
            drivetrain.driveRobotRelative(new ChassisSpeeds());
            return;
        }

        // Get current robot pose using HYBRID approach:
        // - Position from Limelight (more accurate when tag visible)
        // - Rotation from gyro (more stable)
        Pose2d currentPose = getHybridPose();
        boolean usingHybrid = LimelightHelpers.getTV(Constants.limelightName);
        SmartDashboard.putBoolean("PathFindToTag/UsingHybridPose", usingHybrid);

        // Continuously update start position for replanning
        Pathfinding.setStartPosition(currentPose.getTranslation());

        // Check for new path from pathfinder
        if (Pathfinding.isNewPathAvailable()) {
            currentPath = Pathfinding.getCurrentPath(kConstraints, goalEndState);
            
            if (currentPath != null) {
                // Generate trajectory from the path
                // This includes timing information for smooth motion
                ChassisSpeeds currentSpeeds = drivetrain.getRobotRelativeSpeeds();
                currentTrajectory = currentPath.generateTrajectory(
                    currentSpeeds,
                    currentPose.getRotation(),
                    robotConfig
                );
                
                // Restart timer for trajectory sampling
                pathTimer.restart();
                
                SmartDashboard.putString("PathFindToTag/Status", "Following path");
            }
        }

        // Follow the current trajectory
        if (currentTrajectory != null) {
            double time = pathTimer.get();
            
            // Sample the trajectory at current time
            PathPlannerTrajectoryState targetState = currentTrajectory.sample(time);

            // Calculate robot-relative speeds using the holonomic controller
            // This handles BOTH translation (X, Y) AND rotation (heading)
            ChassisSpeeds speeds = controller.calculateRobotRelativeSpeeds(
                currentPose,
                targetState
            );

            // Drive the robot!
            drivetrain.driveRobotRelative(speeds);

            // Debug output
            SmartDashboard.putNumber("PathFindToTag/Time", time);
            SmartDashboard.putNumber("PathFindToTag/TargetStateX", targetState.pose.getX());
            SmartDashboard.putNumber("PathFindToTag/TargetStateY", targetState.pose.getY());
            SmartDashboard.putNumber("PathFindToTag/TargetStateRotDeg", targetState.pose.getRotation().getDegrees());
            SmartDashboard.putNumber("PathFindToTag/SpeedVX", speeds.vxMetersPerSecond);
            SmartDashboard.putNumber("PathFindToTag/SpeedVY", speeds.vyMetersPerSecond);
            SmartDashboard.putNumber("PathFindToTag/SpeedOmega", speeds.omegaRadiansPerSecond);
        } else {
            // No path yet - stop and wait
            drivetrain.driveRobotRelative(new ChassisSpeeds());
            SmartDashboard.putString("PathFindToTag/Status", "Waiting for path...");
        }

        // Update distance/angle to target
        if (targetPose != null) {
            double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
            double angleDiff = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
            SmartDashboard.putNumber("PathFindToTag/DistanceToTarget", distance);
            SmartDashboard.putNumber("PathFindToTag/AngleDiffDeg", Math.toDegrees(angleDiff));
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        drivetrain.stop();
        pathTimer.stop();

        // LED feedback
        if (!interrupted && hasValidTarget) {
            new SetLedColor(0, 255, 0).schedule();  // Green = success
            SmartDashboard.putString("PathFindToTag/Status", "Completed successfully");
        } else {
            new DefaultLed().schedule();
            SmartDashboard.putString("PathFindToTag/Status", interrupted ? "Interrupted" : "No target");
        }
    }

    @Override
    public boolean isFinished() {
        // No valid target = finish immediately
        if (!hasValidTarget || targetPose == null) {
            return true;
        }

        // Check if we're at the target pose (position AND rotation)
        // Use hybrid pose for consistency with execute()
        Pose2d currentPose = getHybridPose();
        
        double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double angleDiff = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());

        boolean positionOk = distance < kPositionTolerance;
        boolean rotationOk = angleDiff < kRotationTolerance;

        SmartDashboard.putBoolean("PathFindToTag/PositionOK", positionOk);
        SmartDashboard.putBoolean("PathFindToTag/RotationOK", rotationOk);

        // Must be at correct position AND rotation
        return positionOk && rotationOk;
    }

    /**
     * Gets the current robot pose using a hybrid approach:
     * - Position (X, Y) from Limelight when tag is visible (more accurate)
     * - Rotation from gyro/odometry (more stable)
     * Falls back to pure odometry if tag not visible or Limelight pose invalid.
     */
    private Pose2d getHybridPose() {
        if (LimelightHelpers.getTV(Constants.limelightName)) {
            Pose2d llPose = drivetrain.getLLPose();
            Pose2d odomPose = drivetrain.getEstimatedPose();
            
            // Only use hybrid if Limelight pose is valid (not 0,0)
            if (llPose.getX() != 0 || llPose.getY() != 0) {
                return new Pose2d(
                    llPose.getTranslation(),    // Position from Limelight
                    odomPose.getRotation()      // Rotation from gyro
                );
            }
        }
        // Fallback to pure odometry
        return drivetrain.getEstimatedPose();
    }
}

