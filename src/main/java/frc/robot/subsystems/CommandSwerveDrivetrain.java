package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.PathPlannerLogging;
import org.littletonrobotics.junction.Logger;
import frc.robot.generated.Constants;
import frc.robot.generated.LimelightHelpers;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private boolean m_isPathFollowing = false;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    //Boolean Checks
    public boolean pathScheduled = false;
    private boolean driveToTagRunning = false;

    // ProfiledPIDController for smooth rotation with motion profiling
    private final ProfiledPIDController rotationController;

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this
            )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this
            )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this
            )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        
        // Initialize ProfiledPIDController with trapezoidal motion profile
        rotationController = new ProfiledPIDController(
            3.5,  // kP - proportional gain
            0.0,  // kI - integral gain
            0.1,  // kD - derivative gain
            new TrapezoidProfile.Constraints(
                4.0,  // max velocity rad/s
                6.0   // max acceleration rad/s^2
            )
        );
        
        // Enable continuous input for angle wrapping
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Math.toRadians(3.0)); // 3 degree tolerance
        
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configurePathPlanner();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     * unspecified or set to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
     * @param modules Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        
        // Initialize ProfiledPIDController with trapezoidal motion profile
        rotationController = new ProfiledPIDController(
            3.5,  // kP
            0.0,  // kI
            0.1,  // kD
            new TrapezoidProfile.Constraints(4.0, 6.0)
        );
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Math.toRadians(3.0));
        
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configurePathPlanner();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     * unspecified or set to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     * in the form [x, y, theta]ᵀ, with units in meters and radians
     * @param visionStandardDeviation The standard deviation for vision calculation
     * in the form [x, y, theta]ᵀ, with units in meters and radians
     * @param modules Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        
        // Initialize ProfiledPIDController with trapezoidal motion profile
        rotationController = new ProfiledPIDController(
            3.5,  // kP
            0.0,  // kI
            0.1,  // kD
            new TrapezoidProfile.Constraints(4.0, 6.0)
        );
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Math.toRadians(3.0));
        
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configurePathPlanner();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return this.getState().Speeds; // Phoenix 6 provides measured robot-relative speeds
    }

    /** Reset odometry to a given field pose (seed CTRE odometry). */
    public void resetOdometry(Pose2d pose) {
        this.resetPose(pose); // CTRE Tuner method
    }

    public Pose2d getEstimatedPose(){
        return this.getState().Pose;
    }

    public Pose2d getLLPose(){
        return new Pose2d(LimelightHelpers.getBotPose2d(Constants.limelightName).getX(),
                LimelightHelpers.getBotPose2d(Constants.limelightName).getY(),
                LimelightHelpers.getBotPose3d(Constants.limelightName).getRotation().toRotation2d());
    }

    public Pose2d getAprilTagPose(){
        return new Pose2d(LimelightHelpers.getTargetPose3d_RobotSpace(Constants.limelightName).getX(),
                LimelightHelpers.getTargetPose3d_RobotSpace(Constants.limelightName).getY(),
                LimelightHelpers.getTargetPose3d_RobotSpace(Constants.limelightName).getRotation().toRotation2d());
    }

    // Apply the chassis speeds
    private final SwerveRequest.ApplyRobotSpeeds ppApplySpeeds = new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

    // Robot-centric drive request (matches FieldCentric config)
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

    public void driveRobotRelative(ChassisSpeeds speeds){
        this.setControl(ppApplySpeeds.withSpeeds(speeds));
    }

    private void configurePathPlanner(){
        RobotConfig ppRobotConfig;
        try{
            ppRobotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            throw new RuntimeException("Pathplanner Config Missing");
        }

        PPHolonomicDriveController ppController = new PPHolonomicDriveController(
                new PIDConstants(Constants.TRANS_KP, Constants.TRANS_KI, Constants.TRANS_KD),
                new PIDConstants(Constants.ROT_KP, Constants.ROT_KI, Constants.ROT_KD)
        );

        AutoBuilder.configure(
                this::getEstimatedPose, // pose supplier
                this::resetOdometry, // reset odometry
                this::getRobotRelativeSpeeds, // robot-relative speeds supplier
                this::driveRobotRelative, // robot-relative drive consumer
                ppController, // holonomic follower controller
                ppRobotConfig, // robot config from GUI
                () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red, // flip paths on red if needed
                this // this subsystem owns the requirement
        );
    }

    public void stop(){
        this.setControl(ppApplySpeeds.withSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0)));
    }

    /**
     * Drive to AprilTag with optimized path and smooth rotation throughout
     * Uses ProfiledPIDController for motion-profiled rotation
     */
    public Command driveToAprilTag() {
        return run(() -> {
            driveToTagRunning = true;

            // Stop if no tag visible
            if (!LimelightHelpers.getTV(Constants.limelightName)) {
                this.setControl(robotCentricDrive
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0));
                rotationController.reset(getEstimatedPose().getRotation().getRadians());
                return;
            }

            // Get tag position relative to robot
            Pose2d tagRelative = getAprilTagPose();
            double tagX = tagRelative.getX();
            double tagY = tagRelative.getY();
            
            // Calculate distance to tag
            double distance = Math.sqrt(tagX * tagX + tagY * tagY);
            
            // Get current robot heading in field frame
            double currentHeading = getEstimatedPose().getRotation().getRadians();
            
            // Calculate target heading to face the tag
            // Transform tag position to field frame
            double tagFieldX = getEstimatedPose().getX() + 
                             (tagX * Math.cos(currentHeading) - tagY * Math.sin(currentHeading));
            double tagFieldY = getEstimatedPose().getY() + 
                             (tagX * Math.sin(currentHeading) + tagY * Math.cos(currentHeading));
            
            double targetHeading = Math.atan2(
                tagFieldY - getEstimatedPose().getY(),
                tagFieldX - getEstimatedPose().getX()
            );
            
            // Use ProfiledPIDController for smooth rotation with motion profiling
            rotationController.setGoal(targetHeading);
            double rotationalVelocity = rotationController.calculate(currentHeading);
            
            // Calculate optimized translation velocities
            // We want to drive in a smooth arc toward the target
            // Velocity direction should point toward target while considering rotation
            
            double angleToTarget = Math.atan2(tagY, tagX);
            
            // Base forward speed on distance
            double baseSpeed;
            if (distance < 0.5) {
                baseSpeed = 0.4;
            } else if (distance < 1.5) {
                baseSpeed = 1.2;
            } else {
                baseSpeed = 2.0;
            }
            
            // Drive velocity in direction of target with smooth arc
            // Adjust velocity based on how much rotation is needed
            double headingError = Math.abs(targetHeading - currentHeading);
            if (headingError > Math.PI) {
                headingError = 2 * Math.PI - headingError;
            }
            
            // Reduce forward speed when large rotation is needed
            double speedMultiplier = 1.0 - (headingError / Math.PI) * 0.5;
            speedMultiplier = Math.max(0.3, speedMultiplier); // Minimum 30% speed
            
            double vx = baseSpeed * Math.cos(angleToTarget) * speedMultiplier;
            double vy = baseSpeed * Math.sin(angleToTarget) * speedMultiplier;
            
            // Apply speed limits
            double maxSpeed = 2.5;
            double maxRotSpeed = 4.0;
            
            vx = Math.max(-maxSpeed, Math.min(maxSpeed, vx));
            vy = Math.max(-maxSpeed, Math.min(maxSpeed, vy));
            rotationalVelocity = Math.max(-maxRotSpeed, Math.min(maxRotSpeed, rotationalVelocity));

            // Debug output
            SmartDashboard.putNumber("Tag/X", tagX);
            SmartDashboard.putNumber("Tag/Y", tagY);
            SmartDashboard.putNumber("Tag/Distance", distance);
            SmartDashboard.putNumber("Robot/CurrentHeading", Math.toDegrees(currentHeading));
            SmartDashboard.putNumber("Robot/TargetHeading", Math.toDegrees(targetHeading));
            SmartDashboard.putNumber("Robot/HeadingError", Math.toDegrees(headingError));
            SmartDashboard.putNumber("ProfiledPID/Goal", Math.toDegrees(rotationController.getGoal().position));
            SmartDashboard.putNumber("ProfiledPID/Setpoint", Math.toDegrees(rotationController.getSetpoint().position));
            SmartDashboard.putBoolean("ProfiledPID/AtGoal", rotationController.atGoal());
            SmartDashboard.putNumber("Cmd/VX", vx);
            SmartDashboard.putNumber("Cmd/VY", vy);
            SmartDashboard.putNumber("Cmd/VRot", rotationalVelocity);
            SmartDashboard.putNumber("Cmd/SpeedMultiplier", speedMultiplier);

            // Drive with smooth rotation throughout the path
            this.setControl(robotCentricDrive
                    .withVelocityX(vx)
                    .withVelocityY(vy)
                    .withRotationalRate(rotationalVelocity));
        })
        .finallyDo(() -> {
            driveToTagRunning = false;
            rotationController.reset(getEstimatedPose().getRotation().getRadians());
        });
    }

    public Command pathFindToPose(Pose2d targetPose, PathConstraints constraints){
        resetPose(getLLPose());
        return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
    }

    public boolean isAtTarget() {
        Pose2d tagRelative = getAprilTagPose();
        double distance = Math.sqrt(tagRelative.getX() * tagRelative.getX() + tagRelative.getY() * tagRelative.getY());
        return distance < 1.0; // Within 1 meter of target
    }

    public boolean setTolerance(){
        return Math.abs(LimelightHelpers.getTargetPose3d_RobotSpace(Constants.limelightName).getX() - LimelightHelpers.getBotPose2d(Constants.limelightName).getX()) < 1 &&
                Math.abs(LimelightHelpers.getTargetPose3d_RobotSpace(Constants.limelightName).getY() - LimelightHelpers.getBotPose2d(Constants.limelightName).getY()) < 1 &&
                Math.abs(LimelightHelpers.getTargetPose3d_RobotSpace(Constants.limelightName).getRotation().toRotation2d().getDegrees() - LimelightHelpers.getBotPose3d(Constants.limelightName).getRotation().toRotation2d().getDegrees()) < 2;
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param requestSupplier Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        SmartDashboard.putNumber("Robot X", getLLPose().getX());
        SmartDashboard.putNumber("Robot Y", getLLPose().getY());
        SmartDashboard.putNumber("Tag X", getAprilTagPose().getX());
        SmartDashboard.putNumber("Tag Y", getAprilTagPose().getY());
        SmartDashboard.putBoolean("Path Scheduled?", pathScheduled);

        // Drive to tag command status
        SmartDashboard.putBoolean("DriveToTag/Running", driveToTagRunning);
        SmartDashboard.putBoolean("DriveToTag/TargetVisible", LimelightHelpers.getTV(Constants.limelightName));

        //Logging
        Logger.recordOutput("RobotPose", getEstimatedPose());

        /* ===================== CORE ODOMETRY ===================== */
        Logger.recordOutput("Drivetrain/Pose", getEstimatedPose());
        Logger.recordOutput("Drivetrain/GyroYaw", getEstimatedPose().getRotation());

        /* ===================== CHASSIS SPEEDS ===================== */
        Logger.recordOutput("Drivetrain/MeasuredSpeeds", getRobotRelativeSpeeds());

        /* ===================== SWERVE MODULE STATES ===================== */
        Logger.recordOutput("Drivetrain/ModuleStates", getState().ModuleStates);

        /* ===================== VISION ===================== */
        Logger.recordOutput("Vision/LimelightPose", getLLPose());
        Logger.recordOutput("Vision/AprilTagPose", getAprilTagPose());

        /* Odometry vs Vision delta (debug fusion) */
        Logger.recordOutput(
                "Vision/OdometryDelta",
                getEstimatedPose().relativeTo(getLLPose())
        );

        /* ===================== MATCH STATE ===================== */
        Logger.recordOutput("Match/Enabled", DriverStation.isEnabled());
        Logger.recordOutput("Match/Alliance", DriverStation.getAlliance().toString());

        if (getAprilTagPose() != null || getAprilTagPose().getX() != 0.0){
            SmartDashboard.putBoolean("Path Scheduled?", pathScheduled);
        } else{
            pathScheduled = false;
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     * in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }
}