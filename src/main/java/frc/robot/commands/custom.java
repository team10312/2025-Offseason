package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.generated.Constants;
import frc.robot.generated.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Command to drive to an AprilTag using PathPlanner.
 * 
 * Pattern used by many FRC teams:
 * - Wrapper command does NOT require the subsystem
 * - Inner PathPlanner command handles subsystem requirement
 * - This avoids scheduling conflicts
 */
public class custom extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private Command pathCommand = null;
    private boolean finished = false;

    public custom(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        // NOTE: Do NOT add requirements here - the pathCommand will handle it
    }

    @Override
    public void initialize() {
        pathCommand = null;
        finished = false;
        
        DriverStation.reportWarning("[custom] initialize() starting", false);

        // Check if tag is visible
        boolean tv = LimelightHelpers.getTV(Constants.limelightName);
        SmartDashboard.putBoolean("custom/tv", tv);
        
        if (!tv) {
            DriverStation.reportWarning("[custom] No tag visible - finishing", false);
            finished = true;
            return;
        }

        // YOUR LOGIC
        Pose2d currentPose = drivetrain.getEstimatedPose();
        Pose2d tagPose = drivetrain.getAprilTagPose();

        // Log to SmartDashboard (more reliable than console)
        SmartDashboard.putNumber("custom/currentX", currentPose.getX());
        SmartDashboard.putNumber("custom/currentY", currentPose.getY());
        SmartDashboard.putNumber("custom/tagX", tagPose.getX());
        SmartDashboard.putNumber("custom/tagY", tagPose.getY());
        
        DriverStation.reportWarning("[custom] tagPose: X=" + tagPose.getX() + " Y=" + tagPose.getY(), false);

        Rotation2d goalRotation = tagPose.getTranslation().getAngle()
            .plus(currentPose.getRotation());

        Pose2d targetPose = currentPose.transformBy(
            new Transform2d(
                tagPose.getTranslation(),
                new Rotation2d()
            )
        );

        Pose2d finalTargetPose = new Pose2d(
            targetPose.getTranslation(),
            goalRotation
        );

        // Calculate distance to target
        double distance = currentPose.getTranslation().getDistance(finalTargetPose.getTranslation());
        SmartDashboard.putNumber("custom/targetX", finalTargetPose.getX());
        SmartDashboard.putNumber("custom/targetY", finalTargetPose.getY());
        SmartDashboard.putNumber("custom/distance", distance);
        
        DriverStation.reportWarning("[custom] distance to target: " + distance + "m", false);

        // Create and schedule the PathPlanner command
        pathCommand = AutoBuilder.pathfindToPose(
            finalTargetPose,
            Constants.constraints,
            0.0
        );

        pathCommand.schedule();
        DriverStation.reportWarning("[custom] pathCommand scheduled", false);
    }

    @Override
    public void execute() {
        // Nothing to do - pathCommand runs independently
    }

    @Override
    public void end(boolean interrupted) {
        DriverStation.reportWarning("[custom] end() interrupted=" + interrupted + 
            ", pathCommand=" + (pathCommand != null ? "exists" : "null") +
            ", wasScheduled=" + (pathCommand != null && pathCommand.isScheduled()), false);
        
        if (pathCommand != null && pathCommand.isScheduled()) {
            pathCommand.cancel();
        }
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        // Finish if no tag was visible
        if (finished) {
            DriverStation.reportWarning("[custom] isFinished: no tag", false);
            return true;
        }
        
        // pathCommand not created yet
        if (pathCommand == null) {
            DriverStation.reportWarning("[custom] isFinished: pathCommand is null!", false);
            return true;
        }
        
        // Finish when pathCommand is done
        if (!pathCommand.isScheduled()) {
            DriverStation.reportWarning("[custom] isFinished: pathCommand not scheduled (finished or failed)", false);
            return true;
        }
        
        return false;
    }
}
