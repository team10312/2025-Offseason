package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.generated.Constants;
import frc.robot.generated.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class custom {

    /**
     * Creates a command to drive to an AprilTag using PathPlanner.
     * Uses command composition to avoid subsystem conflicts.
     */
    public static Command create(CommandSwerveDrivetrain drivetrain) {
        return Commands.defer(() -> {
            // Check if tag is visible
            if (!LimelightHelpers.getTV(Constants.limelightName)) {
                System.out.println("[custom] No tag visible");
                return Commands.none();
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

            // Return the PathPlanner command directly
            return AutoBuilder.pathfindToPose(
                finalTargetPose,
                Constants.constraints,
                0.0
            ).finallyDo(() -> drivetrain.stop());
            
        }, drivetrain);
    }
}
