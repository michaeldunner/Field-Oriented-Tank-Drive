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

  public Command updateDriveMode(DriveMode driveMode) {
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

    final double DEADBAND = 0.1;

    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
    Logger.recordOutput("Drive/linearVelocity", linearVelocity);

    // Apply rotation deadband
    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

    // Square rotation value for more precise control
    omega = Math.copySign(omega * omega, omega);

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    double flip = isFlipped ? -1.0 : 1.0;

    Rotation2d robotDirection = drive.getRotation();

    double joyStickOmega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), Constants.DEADBAND);

    ChassisSpeeds speeds =
        new ChassisSpeeds(linearVelocity.getX() * flip, linearVelocity.getY() * flip, omega);

    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.dtSeconds);

    Rotation2d stickDirection =
        new Rotation2d(discreteSpeeds.vxMetersPerSecond, discreteSpeeds.vyMetersPerSecond);

    double forwardSpeed =
        Math.hypot(discreteSpeeds.vxMetersPerSecond, discreteSpeeds.vyMetersPerSecond);

    double thetaError =
        MathUtil.angleModulus(stickDirection.getRadians() - robotDirection.getRadians());

    // If stick direction is more than 90° from robot heading, drive backward instead
    double setpoint = stickDirection.getRadians();
    if (Math.abs(thetaError) > (Math.PI / 2.0)) {
      setpoint = MathUtil.angleModulus(setpoint + Math.PI);
      forwardSpeed = -forwardSpeed;
      thetaError = MathUtil.angleModulus(setpoint - robotDirection.getRadians());
    }

    DifferentialDrive.WheelSpeeds fieldOrientedSpeeds = new DifferentialDrive.WheelSpeeds();

    Translation2d target;
    double shootingAngle;

    Logger.recordOutput("Drive/driveMode", currentDriveMode);

    switch (currentDriveMode) {
      case NORMAL:
        fieldOrientedSpeeds = DifferentialDrive.arcadeDriveIK(forwardSpeed, joyStickOmega, false);

        if (Math.abs(thetaError) > DriveConstants.turnToleranceRad
            && Math.abs(forwardSpeed) > 0.01) {
          fieldOrientedSpeeds =
              DifferentialDrive.arcadeDriveIK(
                  forwardSpeed,
                  rotationController.calculate(robotDirection.getRadians(), setpoint),
                  false);
        }

        break;

      case TRENCH_LOCK:
        double trenchYError =
            rotationController.calculate(drive.getPose().getY(), getTrenchY().in(Meters));
        Logger.recordOutput("Drive/trenchYError", trenchYError);

        double centeringCorrection = (getTrenchY().in(Meters) - drive.getPose().getY());

        double snapAngle = getTrenchLockAngle().getRadians();

        double headingFactor = Math.cos(snapAngle);

        fieldOrientedSpeeds =
            DifferentialDrive.arcadeDriveIK(
                forwardSpeed,
                rotationController.calculate(
                    robotDirection.getRadians(),
                    snapAngle
                        + (centeringCorrection
                            * DriveConstants.driveAssistTrenchCenteringCorrectionFactor
                            * Math.signum(forwardSpeed)
                            * headingFactor)),
                false);

        break;

        // shooting and passing are the same mode, they just have different targets
      case SHOOTING:
        target = FieldConstants.getTurretTarget(drive.getPose());

        shootingAngle =
            Math.atan2(
                    target.getY() - drive.getPose().getY(), target.getX() - drive.getPose().getX())
                + Math.PI;

        fieldOrientedSpeeds =
            DifferentialDrive.arcadeDriveIK(
                0.0,
                rotationController.calculate(robotDirection.getRadians(), shootingAngle),
                false);
        break;
    }
    drive.runClosedLoop(
        fieldOrientedSpeeds.left * DriveConstants.maxSpeedMetersPerSec,
        fieldOrientedSpeeds.right * DriveConstants.maxSpeedMetersPerSec);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public enum DriveMode {
    NORMAL,
    TRENCH_LOCK,
    SHOOTING
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
