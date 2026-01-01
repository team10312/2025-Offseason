package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.generated.Constants;
import frc.robot.generated.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class custom extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private Command pathCommand = null;
    private boolean shouldFinishImmediately = false;

    public custom(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        pathCommand = null;
        shouldFinishImmediately = false;

        // Check if tag is visible (tv = target valid)
        if (!LimelightHelpers.getTV(Constants.limelightName)) {
            shouldFinishImmediately = true;
            return;
        }

        // YOUR LOGIC
        Pose2d currentPose = drivetrain.getEstimatedPose();
        Pose2d tagPose = drivetrain.getAprilTagPose();

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

        // PathPlanner handles completion internally
        pathCommand = AutoBuilder.pathfindToPose(
            finalTargetPose,
            Constants.constraints,
            0.0
        );

        pathCommand.schedule();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        if (pathCommand != null && pathCommand.isScheduled()) {
            pathCommand.cancel();
        }
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        if (shouldFinishImmediately) {
            return true;
        }
        return pathCommand != null && !pathCommand.isScheduled();
    }
}
