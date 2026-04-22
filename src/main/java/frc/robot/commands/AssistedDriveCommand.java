// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.FieldConstants;
import frc.robot.util.GeometryUtil;
import frc.robot.util.Zones;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

/** Default drive command to run that drives based on controller input */
public class AssistedDriveCommand extends Command {
  private final Drive drive = Drive.getInstance();
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;
  private int flipFactor = 1; // 1 for normal, -1 for flipped

  @AutoLogOutput private final Trigger inTrenchZoneTrigger;

  private final PIDController rotationController =
      new PIDController(DriveConstants.turnKp, DriveConstants.turnKi, DriveConstants.turnKd);

  @AutoLogOutput private DriveMode currentDriveMode = DriveMode.NORMAL;

  /** Creates a new TeleopDrive. */
  public AssistedDriveCommand(CommandPS5Controller controller) {
    this.xSupplier = () -> -controller.getLeftY() * flipFactor;
    this.ySupplier = () -> -controller.getLeftX() * flipFactor;
    this.omegaSupplier = () -> -controller.getRightX();

    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    inTrenchZoneTrigger =
        Zones.trenchZones
            .willContain(
                drive::getPose,
                drive::getFieldRelativeChassisSpeeds,
                Seconds.of(DriveConstants.trenchAlignTimeSeconds))
            .debounce(0.1);

    inTrenchZoneTrigger.onTrue(updateDriveMode(DriveMode.TRENCH_LOCK));

    inTrenchZoneTrigger.onFalse(updateDriveMode(DriveMode.NORMAL));

    addRequirements(drive);

    Zones.logAllZones();
    AutoLogOutputManager.addObject(this);
  }

  private Distance getTrenchY() {
    Pose2d robotPose = drive.getPose();
    if (robotPose.getMeasureY().gte(Meters.of(FieldConstants.fieldWidth / 2))) {
      return Meters.of(FieldConstants.LeftTrench.center.getY());
    }
    return Meters.of(FieldConstants.RightTrench.center.getY());
  }

  private Rotation2d getTrenchLockAngle() {
    return GeometryUtil.getNearest180Rotation(drive.getRotation());
  }

  private Command updateDriveMode(DriveMode driveMode) {
    return Commands.runOnce(
        () -> {
          currentDriveMode = driveMode;
          rotationController.reset();
        });
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentDriveMode = DriveMode.NORMAL;
    rotationController.reset();
    flipFactor =
        DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red
            ? -1
            : 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Translation2d linearVelocity =
    //     getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
    // linearVelocity = linearVelocity.times(driveSubsystem.getMaxLinearSpeedMetersPerSec());
    // linearVelocity = driveLimiter.calculate(linearVelocity);

    // double omega =
    //     MathUtil.applyDeadband(omegaSupplier.getAsDouble(),
    // ControllerConstants.controllerDeadband);
    // omega = Math.copySign(omega * omega, omega); // square for more precise rotation control

    final double DEADBAND = 0.1;

    switch (currentDriveMode) {
      case NORMAL:
        // Get linear velocity = get linear velocity
        Translation2d linearVelocity =
            getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
        Logger.recordOutput("Drive/linearVelocity", linearVelocity);
        // Apply rotation deadband
        double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

        // Square rotation value for more precise control
        omega = Math.copySign(omega * omega, omega);
        // Flip joystick direction for Red alliance
        // (negate vx/vy to rotate 180°, so "forward" points toward Red wall)
        boolean isFlipped =
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
        double flip = isFlipped ? -1.0 : 1.0;

        // Pass field-relative speeds directly to runVelocity
        // (runVelocity handles field-centric turning internally)
        ChassisSpeeds speeds =
            new ChassisSpeeds(linearVelocity.getX() * flip, linearVelocity.getY() * flip, omega);

        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.dtSeconds);
        // only look at omega controller if the other 2 velocites are zero

        // get stick direction and pid to direction and then drive both motors forward

        // runClosedLoop();

        PIDController rotationPID =
            new PIDController(DriveConstants.turnKp, DriveConstants.turnKi, DriveConstants.turnKd);
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        Rotation2d stickDirection =
            new Rotation2d(discreteSpeeds.vxMetersPerSecond, discreteSpeeds.vyMetersPerSecond);
        //         .plus(
        //             new Rotation2d(
        //                 Math.PI / 2.0)); // have to add pi/2 because the stick direction is -
        // pi/2
        // when
        // // pushed forward but we want that to be 0 in the robot frame
        Rotation2d robotDirection = drive.getRotation();

        Logger.recordOutput("Drive/DiscreteSpeeds", discreteSpeeds);
        Logger.recordOutput("Drive/stickDirection", stickDirection);
        Logger.recordOutput("Drive/robotDirection", robotDirection);

        double forwardSpeed =
            Math.hypot(discreteSpeeds.vxMetersPerSecond, discreteSpeeds.vyMetersPerSecond);
        Logger.recordOutput("Drive/forwardSpeed", forwardSpeed);

        double joyStickOmega =
            MathUtil.applyDeadband(omegaSupplier.getAsDouble(), Constants.DEADBAND);

        // Compute shortest angular difference (properly wrapped to [-π, π])
        double thetaError =
            MathUtil.angleModulus(stickDirection.getRadians() - robotDirection.getRadians());

        // If stick direction is more than 90° from robot heading, drive backward instead
        double setpoint = stickDirection.getRadians();
        if (Math.abs(thetaError) > (Math.PI / 2.0)) {
          setpoint = MathUtil.angleModulus(setpoint + Math.PI);
          forwardSpeed = -forwardSpeed;
        }

        // Default: drive at forwardSpeed with joystick omega
        var fieldOrientedSpeeds =
            DifferentialDrive.arcadeDriveIK(forwardSpeed, joyStickOmega, false);

        // PID turns until angle is within tolerance
        if (Math.abs(thetaError) > DriveConstants.turnToleranceRad
            && Math.abs(forwardSpeed) > 0.01) {
          fieldOrientedSpeeds =
              DifferentialDrive.arcadeDriveIK(
                  forwardSpeed,
                  rotationPID.calculate(robotDirection.getRadians(), setpoint),
                  false);
        }

        drive.runClosedLoop(
            fieldOrientedSpeeds.left * DriveConstants.maxSpeedMetersPerSec,
            fieldOrientedSpeeds.right * DriveConstants.maxSpeedMetersPerSec);
        break;
      case TRENCH_LOCK:
        // Get linear velocity = get linear velocity
        Translation2d trenchLinearVelocity =
            getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
        Logger.recordOutput("Drive/trenchLinearVelocity", trenchLinearVelocity);
        // Apply rotation deadband
        double trenchOmega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

        // Square rotation value for more precise control
        trenchOmega = Math.copySign(trenchOmega * trenchOmega, trenchOmega);
        // Flip joystick direction for Red alliance
        // (negate vx/vy to rotate 180°, so "forward" points toward Red wall)
        boolean trenchIsFlipped =
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
        double trenchFlip = trenchIsFlipped ? -1.0 : 1.0;

        // Pass field-relative speeds directly to runVelocity
        // (runVelocity handles field-centric turning internally)
        ChassisSpeeds trenchSpeeds =
            new ChassisSpeeds(
                trenchLinearVelocity.getX() * trenchFlip,
                trenchLinearVelocity.getY() * trenchFlip,
                trenchOmega);

        ChassisSpeeds trenchDiscreteSpeeds =
            ChassisSpeeds.discretize(trenchSpeeds, Constants.dtSeconds);
        // only look at omega controller if the other 2 velocites are zero

        // get stick direction and pid to direction and then drive both motors forward

        // runClosedLoop();

        PIDController trenchRotationPID =
            new PIDController(
                DriveConstants.driveAssistKp,
                DriveConstants.driveAssistKi,
                DriveConstants.driveAssistKd);
        trenchRotationPID.enableContinuousInput(-Math.PI, Math.PI);
        Rotation2d trenchStickDirection =
            new Rotation2d(
                trenchDiscreteSpeeds.vxMetersPerSecond, trenchDiscreteSpeeds.vyMetersPerSecond);
        //         .plus(
        //             new Rotation2d(
        //                 Math.PI / 2.0)); // have to add pi/2 because the stick direction is -
        // pi/2
        // when
        // // pushed forward but we want that to be 0 in the robot frame
        Rotation2d trenchRobotDirection = drive.getRotation();

        Logger.recordOutput("Drive/DiscreteSpeeds", trenchDiscreteSpeeds);
        Logger.recordOutput("Drive/trenchStickDirection", trenchStickDirection);
        Logger.recordOutput("Drive/trenchRobotDirection", trenchRobotDirection);
        double trenchForwardSpeed =
            Math.hypot(
                trenchDiscreteSpeeds.vxMetersPerSecond, trenchDiscreteSpeeds.vyMetersPerSecond);
        Logger.recordOutput("Drive/trenchForwardSpeed", trenchForwardSpeed);

        double trenchJoyStickOmega =
            MathUtil.applyDeadband(omegaSupplier.getAsDouble(), Constants.DEADBAND);

        // Compute shortest angular difference (properly wrapped to [-π, π])
        double trenchThetaError =
            MathUtil.angleModulus(
                trenchStickDirection.getRadians() - trenchRobotDirection.getRadians());
        // If stick direction is more than 90° from robot heading, drive backward instead
        double trenchSetpoint = trenchStickDirection.getRadians();
        if (Math.abs(trenchThetaError) > (Math.PI / 2.0)) {
          trenchSetpoint = MathUtil.angleModulus(trenchSetpoint + Math.PI);
          trenchForwardSpeed = -trenchForwardSpeed;
        }

        // Default: drive at forwardSpeed with joystick omega
        var trenchFieldOrientedSpeeds =
            DifferentialDrive.arcadeDriveIK(trenchForwardSpeed, trenchJoyStickOmega, false);

        double trenchYError =
            rotationController.calculate(drive.getPose().getY(), getTrenchY().in(Meters));
        Logger.recordOutput("Drive/trenchYError", trenchYError);

        double centeringCorrection = (drive.getPose().getY() - getTrenchY().in(Meters));

        double snapAngle = getTrenchLockAngle().getRadians();
        // +1 when facing 0°, -1 when facing 180° — flips correction
        // since robot-right maps to opposite field-Y directions at each heading
        double headingFactor = Math.cos(snapAngle);

        // PID turns until angle is within tolerance
        if (Math.abs(trenchThetaError) > DriveConstants.turnToleranceRad) {
          trenchFieldOrientedSpeeds =
              DifferentialDrive.arcadeDriveIK(
                  trenchForwardSpeed,
                  trenchRotationPID.calculate(
                      trenchRobotDirection.getRadians(),
                      snapAngle
                          + (centeringCorrection
                              * DriveConstants.driveAssistTrenchCenteringCorrectionFactor
                              * Math.signum(trenchForwardSpeed)
                              * headingFactor
                              * -0.5)),
                  false);
        }

        drive.runClosedLoop(
            trenchFieldOrientedSpeeds.left * DriveConstants.maxSpeedMetersPerSec,
            trenchFieldOrientedSpeeds.right * DriveConstants.maxSpeedMetersPerSec);
        // rotationController.setSetpoint(getTrenchY().in(Meters));
        // // Clamp the y velocity to the max linear speed
        // double yVel =
        //     MathUtil.clamp(
        //         rotationController.calculate(driveSubsystem.getPose().getY()),
        //         -driveSubsystem.getMaxLinearSpeedMetersPerSec(),
        //         driveSubsystem.getMaxLinearSpeedMetersPerSec());
        // if (rotationController.atSetpoint()) {
        //   yVel = 0;
        // }
        // rotationController.setSetpoint(getTrenchLockAngle().getRadians());
        // // Clamp the rotation speed to the max angular speed
        // double rotSpeedToStraight =
        //     MathUtil.clamp(
        //         rotationController.calculate(driveSubsystem.getRotation().getRadians()),
        //         -driveSubsystem.getMaxAngularSpeedRadPerSec(),
        //         driveSubsystem.getMaxAngularSpeedRadPerSec());
        // if (rotationController.atSetpoint()) {
        //   rotSpeedToStraight = 0;
        // }
        // driveSubsystem.runVelocityFieldRelative(
        //     new ChassisSpeeds(
        //         MetersPerSecond.of(linearVelocity.getX()),
        //         MetersPerSecond.of(yVel),
        //         RadiansPerSecond.of(rotSpeedToStraight)));
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private enum DriveMode {
    NORMAL,
    TRENCH_LOCK,
    BUMP_LOCK,
    TOWER_LOCK,
    HUB_LOCK
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), Constants.DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));
    Logger.recordOutput("Drive/joystickDirection", linearDirection);

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }
}
