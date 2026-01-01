package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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

        // Check if tag is visible
        if (!LimelightHelpers.getTV(Constants.limelightName)) {
            System.out.println("[custom] No tag visible - finishing");
            finished = true;
            return;
        }

        // YOUR LOGIC
        Pose2d currentPose = drivetrain.getEstimatedPose();
        Pose2d tagPose = drivetrain.getAprilTagPose();

        System.out.println("[custom] currentPose: " + currentPose);
        System.out.println("[custom] tagPose: " + tagPose);

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

        System.out.println("[custom] finalTargetPose: " + finalTargetPose);

        // Create and schedule the PathPlanner command
        pathCommand = AutoBuilder.pathfindToPose(
            finalTargetPose,
            Constants.constraints,
            0.0
        );

        pathCommand.schedule();
        System.out.println("[custom] pathCommand scheduled");
    }

    @Override
    public void execute() {
        // Nothing to do - pathCommand runs independently
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("[custom] end() called, interrupted=" + interrupted);
        
        if (pathCommand != null && pathCommand.isScheduled()) {
            pathCommand.cancel();
        }
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        // Finish if no tag was visible
        if (finished) {
            return true;
        }
        
        // Finish when pathCommand is done
        if (pathCommand != null && !pathCommand.isScheduled()) {
            System.out.println("[custom] pathCommand finished");
            return true;
        }
        
        return false;
    }
}
