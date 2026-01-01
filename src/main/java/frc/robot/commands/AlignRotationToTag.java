package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.generated.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignRotationToTag extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final String limelightName;

  private final SwerveRequest.RobotCentric req =
      new SwerveRequest.RobotCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // Tune these
  private final PIDController rotPID = new PIDController(5.0, 0.0, 0.25);

  private final double maxOmegaRadPerSec = 6.0;
  private final double tolRad = Units.degreesToRadians(2.0);

  public AlignRotationToTag(CommandSwerveDrivetrain drivetrain, String limelightName) {
    this.drivetrain = drivetrain;
    this.limelightName = limelightName;
    addRequirements(drivetrain);

    rotPID.enableContinuousInput(-Math.PI, Math.PI);
    rotPID.setTolerance(tolRad);
  }

  @Override
  public void initialize() {
    rotPID.reset();
    rotPID.setSetpoint(0.0); // <-- what you asked for
    SmartDashboard.putBoolean("AlignRot/AtTarget", false);
  }

  @Override
  public void execute() {
    boolean hasTarget = LimelightHelpers.getTV(limelightName);
    SmartDashboard.putBoolean("AlignRot/HasTarget", hasTarget);

    if (!RobotBase.isSimulation() && !hasTarget) {
      // No target -> stop rotating (and DEFINITELY no translation)
      drivetrain.setControl(req.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
      rotPID.reset();
      rotPID.setSetpoint(0.0);
      SmartDashboard.putBoolean("AlignRot/AtTarget", false);
      return;
    }

    // Measurement = angle-to-tag in robot frame (radians). Goal is 0.
    double angleErrRad;
    if (RobotBase.isSimulation()) {
      angleErrRad = Units.degreesToRadians(SmartDashboard.getNumber("AlignRot/SimErrDeg", 15.0));
    } else {
      Pose2d tagRel = drivetrain.getAprilTagPose();
      angleErrRad = tagRel.getTranslation().getAngle().getRadians();
    }

    // PID drives measurement -> setpoint (0)
    double omega = rotPID.calculate(angleErrRad);

    // tiny deadband to stop buzzing
    omega = MathUtil.applyDeadband(omega, 0.02);
    omega = MathUtil.clamp(omega, -maxOmegaRadPerSec, maxOmegaRadPerSec);

    // PURE ROTATION: vx=0, vy=0 ALWAYS (prevents stale drift)
    drivetrain.setControl(
        req.withVelocityX(0)
           .withVelocityY(0)
           .withRotationalRate(omega)
    );

    boolean atTarget = rotPID.atSetpoint();
    SmartDashboard.putNumber("AlignRot/ErrorDeg", Units.radiansToDegrees(angleErrRad));
    SmartDashboard.putBoolean("AlignRot/AtTarget", atTarget);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(req.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    SmartDashboard.putBoolean("AlignRot/AtTarget", false);
  }

  @Override
  public boolean isFinished() {
    // usually you run this while-held
    return false;
  }
}
