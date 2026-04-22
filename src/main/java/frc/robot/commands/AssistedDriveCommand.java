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

  private final PIDController trenchYController =
      new PIDController(
          DriveConstants.driveAssistKp, DriveConstants.driveAssistKi, DriveConstants.driveAssistKd);

  //   private final PIDController towerXController =
  //       new PIDController(
  //           DriveConstants.driveAssistTranslationKp,
  //           DriveConstants.driveAssistTranslationKi,
  //           DriveConstants.driveAssistTranslationKd);
  //   private final PIDController hubXController =
  //       new PIDController(
  //           DriveConstants.driveAssistTranslationKp,
  //           DriveConstants.driveAssistTranslationKi,
  //           DriveConstants.driveAssistTranslationKd);

  //   private final PIDController rotationController =
  //       new PIDController(
  //           DriveConstants.driveAssistRotationKp,
  //           DriveConstants.driveAssistRotationKi,
  //           DriveConstants.driveAssistRotationKd);

  @AutoLogOutput private DriveMode currentDriveMode = DriveMode.NORMAL;

  /** Creates a new TeleopDrive. */
  public AssistedDriveCommand(CommandPS5Controller controller) {
    this.xSupplier = () -> -controller.getLeftY() * flipFactor;
    this.ySupplier = () -> -controller.getLeftX() * flipFactor;
    this.omegaSupplier = () -> -controller.getRightX();

    // trenchYController.setTolerance(DriveConstants.trenchAlignPositionTolerance);
    trenchYController.enableContinuousInput(-Math.PI, Math.PI);
    // towerXController.setTolerance(DriveConstants.towerAlignPositionTolerance);
    // hubXController.setTolerance(DriveConstants.hubAlignPositionTolerance);
    // rotationController.setTolerance(DriveConstants.rotationAlignTolerance);
    // rotationController.enableContinuousInput(-Math.PI, Math.PI);

    // Trigger notSprinting = ControllerUtil.sprintToggle(controller).negate();
    // Trigger noOverridesActive =
    //     notSprinting
    //         .and(() -> !DriverStation.isTest())
    //         .and(OverrideUtil.isManualModeTrigger().negate());

    inTrenchZoneTrigger =
        // noOverridesActive.and(
        Zones.trenchZones
            .willContain(
                drive::getPose,
                drive::getFieldRelativeChassisSpeeds,
                Seconds.of(DriveConstants.trenchAlignTimeSeconds))
            .debounce(0.1);
    // .debounce(0.1));

    // inTowerZoneTrigger = new Trigger(() -> false);
    // noOverridesActive.and(
    //     Zones.towerZones
    //         .willContain(
    //             driveSubsystem::getPose,
    //             driveSubsystem::getFieldRelativeChassisSpeeds,
    //             Seconds.of(DriveConstants.towerAlignTimeSeconds))
    //         .debounce(0.1));

    // inHubDropAreaTrigger =
    //     noOverridesActive.and(
    //         Zones.hubDropAreas
    //             .willContain(
    //                 driveSubsystem::getPose,
    //                 driveSubsystem::getFieldRelativeChassisSpeeds,
    //                 Seconds.of(DriveConstants.hubDropAreaTimeSeconds))
    //             .debounce(0.1));

    // inBumpZoneTrigger =
    //     noOverridesActive
    //         .and(
    //             Zones.bumpZones
    //                 .willContain(
    //                     driveSubsystem::getPose,
    //                     driveSubsystem::getFieldRelativeChassisSpeeds,
    //                     Seconds.of(DriveConstants.bumpAlignTimeSeconds))
    //                 .debounce(0.1))
    //         // negate hub area trigger so that hub and bump don't fight each other
    //         .and(inHubDropAreaTrigger.negate());

    inTrenchZoneTrigger.onTrue(updateDriveMode(DriveMode.TRENCH_LOCK));
    // inBumpZoneTrigger.onTrue(updateDriveMode(DriveMode.BUMP_LOCK));
    // inTowerZoneTrigger.onTrue(updateDriveMode(DriveMode.TOWER_LOCK));
    // inHubDropAreaTrigger.onTrue(updateDriveMode(DriveMode.HUB_LOCK));
    inTrenchZoneTrigger
        // .or(inBumpZoneTrigger)
        // .or(inTowerZoneTrigger)
        // .or(inHubDropAreaTrigger)
        .onFalse(updateDriveMode(DriveMode.NORMAL));

    addRequirements(drive);

    Zones.logAllZones();
    AutoLogOutputManager.addObject(this);
  }

  // private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
  //   // Apply deadband
  //   double linearMagnitude =
  //       MathUtil.applyDeadband(Math.hypot(x, y), ControllerConstants.controllerDeadband);
  //   Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

  //   // Square magnitude for more precise control
  //   linearMagnitude = linearMagnitude * linearMagnitude;

  //   // Return new linear velocity
  //   return new Translation2d(linearMagnitude, linearDirection);
  // }

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

  // private Distance getTowerX() {
  //   if (driveSubsystem.getPose().getX() < FieldConstants.fieldLength / 2) {
  //     return Meters.of(FieldConstants.Tower.depth / 2);
  //   }
  //   return Meters.of(FieldConstants.fieldLength - FieldConstants.Tower.depth / 2);
  // }

  // private Distance getHubX() {
  //   if (driveSubsystem.getPose().getX() < FieldConstants.fieldLength / 2) {
  //     return Meters.of(FieldConstants.LinesVertical.hubCenter + FieldConstants.Hub.width);
  //   }
  //   return Meters.of(FieldConstants.LinesVertical.oppHubCenter - FieldConstants.Hub.width);
  // }

  // private Rotation2d getTowerLockAngle() {
  //   return GeometryUtil.getNearest90or270Rotation(driveSubsystem.getRotation());
  // }

  // private Rotation2d getHubLockAngle() {
  //   return GeometryUtil.getNearest90or270Rotation(driveSubsystem.getRotation());
  // }

  // private Rotation2d getBumpLockAngle() {
  //   // Angles where the robot's intake (front, 0°) faces in the ±X direction
  //   Rotation2d[] intakeFirstAngles = {Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(-45)};
  //   // Angles where the robot's intake faces away from the ±X direction (intake trails)
  //   Rotation2d[] intakeTrailingAngles = {Rotation2d.fromDegrees(135),
  // Rotation2d.fromDegrees(-135)};

  //   boolean isIntakeExtended = IntakeSubsystem.getInstance().isExtensionOut();
  //   if (!isIntakeExtended) {
  //     return GeometryUtil.getNearestRotation(
  //         driveSubsystem.getRotation(),
  //         intakeFirstAngles[0],
  //         intakeFirstAngles[1],
  //         intakeTrailingAngles[0],
  //         intakeTrailingAngles[1]);
  //   }

  //   // Intake is down — only allow headings where the intake trails the direction of X travel,
  //   // since we cannot drive over the bump intake-first when it is extended
  //   double vx = driveSubsystem.getFieldRelativeChassisSpeeds().vxMetersPerSecond;
  //   if (vx > 0) {
  //     // Moving in +X: intake must face -X → 135° or -135°
  //     return GeometryUtil.getNearestRotation(
  //         driveSubsystem.getRotation(), intakeTrailingAngles[0], intakeTrailingAngles[1]);
  //   } else if (vx < 0) {
  //     // Moving in -X: intake must face +X → 45° or -45°
  //     return GeometryUtil.getNearestRotation(
  //         driveSubsystem.getRotation(), intakeFirstAngles[0], intakeFirstAngles[1]);
  //   } else {
  //     // No X motion; any heading is fine
  //     return GeometryUtil.getNearestRotation(
  //         driveSubsystem.getRotation(),
  //         intakeFirstAngles[0],
  //         intakeFirstAngles[1],
  //         intakeTrailingAngles[0],
  //         intakeTrailingAngles[1]);
  //   }
  // }

  private Command updateDriveMode(DriveMode driveMode) {
    return Commands.runOnce(
        () -> {
          currentDriveMode = driveMode;
          // rotationController.reset();
          trenchYController.reset();
          // towerXController.reset();
          // hubXController.reset();
        });
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentDriveMode = DriveMode.NORMAL;
    trenchYController.reset();
    // rotationController.reset();
    // driveLimiter.reset(new Translation2d());
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
            trenchYController.calculate(drive.getPose().getY(), getTrenchY().in(Meters));
        Logger.recordOutput("Drive/trenchYError", trenchYError);

        double centeringCorrection = (drive.getPose().getY() - getTrenchY().in(Meters));

        double snapAngle = GeometryUtil.getNearest180Rotation(drive.getRotation()).getRadians();
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
        // trenchYController.setSetpoint(getTrenchY().in(Meters));
        // // Clamp the y velocity to the max linear speed
        // double yVel =
        //     MathUtil.clamp(
        //         trenchYController.calculate(driveSubsystem.getPose().getY()),
        //         -driveSubsystem.getMaxLinearSpeedMetersPerSec(),
        //         driveSubsystem.getMaxLinearSpeedMetersPerSec());
        // if (trenchYController.atSetpoint()) {
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
        // case BUMP_LOCK:
        //   rotationController.setSetpoint(getBumpLockAngle().getRadians());
        //   // Clamp the rotation speed to the max angular speed
        //   double rotSpeedToDiagonal =
        //       MathUtil.clamp(
        //           rotationController.calculate(driveSubsystem.getRotation().getRadians()),
        //           -driveSubsystem.getMaxAngularSpeedRadPerSec(),
        //           driveSubsystem.getMaxAngularSpeedRadPerSec());
        //   if (rotationController.atSetpoint()) {
        //     rotSpeedToDiagonal = 0;
        //   }
        //   driveSubsystem.runVelocityFieldRelative(
        //       new ChassisSpeeds(
        //           MetersPerSecond.of(linearVelocity.getX()),
        //           MetersPerSecond.of(linearVelocity.getY()),
        //           RadiansPerSecond.of(rotSpeedToDiagonal)));
        //   break;

        // case TOWER_LOCK:
        //   towerXController.setSetpoint(getTowerX().in(Meters));
        //   double xVel =
        //       MathUtil.clamp(
        //           towerXController.calculate(driveSubsystem.getPose().getX()),
        //           -driveSubsystem.getMaxLinearSpeedMetersPerSec(),
        //           driveSubsystem.getMaxLinearSpeedMetersPerSec());
        //   if (towerXController.atSetpoint()) {
        //     xVel = 0;
        //   }
        //   rotationController.setSetpoint(getTowerLockAngle().getRadians());
        //   double rotSpeedToSideways =
        //       MathUtil.clamp(
        //           rotationController.calculate(driveSubsystem.getRotation().getRadians()),
        //           -driveSubsystem.getMaxAngularSpeedRadPerSec(),
        //           driveSubsystem.getMaxAngularSpeedRadPerSec());
        //   if (rotationController.atSetpoint()) {
        //     rotSpeedToSideways = 0;
        //   }
        //   driveSubsystem.runVelocityFieldRelative(
        //       new ChassisSpeeds(
        //           MetersPerSecond.of(xVel),
        //           MetersPerSecond.of(linearVelocity.getY()),
        //           RadiansPerSecond.of(rotSpeedToSideways)));
        //   break;

        // case HUB_LOCK:
        //   hubXController.setSetpoint(getHubX().in(Meters));
        //   double xHubVel =
        //       MathUtil.clamp(
        //           hubXController.calculate(driveSubsystem.getPose().getX()),
        //           -driveSubsystem.getMaxLinearSpeedMetersPerSec(),
        //           driveSubsystem.getMaxLinearSpeedMetersPerSec());
        //   if (hubXController.atSetpoint()) {
        //     xHubVel = 0;
        //   }
        //   rotationController.setSetpoint(getHubLockAngle().getRadians());
        //   double rotSpeedToSidewaysHub =
        //       MathUtil.clamp(
        //           rotationController.calculate(driveSubsystem.getRotation().getRadians()),
        //           -driveSubsystem.getMaxAngularSpeedRadPerSec(),
        //           driveSubsystem.getMaxAngularSpeedRadPerSec());
        //   if (rotationController.atSetpoint()) {
        //     rotSpeedToSidewaysHub = 0;
        //   }
        //   driveSubsystem.runVelocityFieldRelative(
        //       new ChassisSpeeds(
        //           MetersPerSecond.of(xHubVel),
        //           MetersPerSecond.of(linearVelocity.getY()),
        //           RadiansPerSecond.of(rotSpeedToSidewaysHub)));

        //   break;
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
