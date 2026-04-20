// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import static frc.robot.subsystems.drive.DriveConstants.maxSpeedMetersPerSec;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec

  private DriveCommands() {}

  /**
   * Standard joystick drive, where X is the forward-backward axis (positive = forward) and Z is the
   * left-right axis (positive = counter-clockwise).
   */
  public static Command arcadeDrive(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier zSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), DEADBAND);
          double z = MathUtil.applyDeadband(zSupplier.getAsDouble(), DEADBAND);

          // Calculate speeds
          var speeds = DifferentialDrive.arcadeDriveIK(x, z, true);

          // Apply output
          drive.runClosedLoop(
              speeds.left * maxSpeedMetersPerSec, speeds.right * maxSpeedMetersPerSec);
        },
        drive);
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));
    Logger.recordOutput("Drive/joystickDirection", linearDirection);

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Field oriented tank drive meant to feel like swerve, controlled like a single swereve module
   */
  public static Command fieldOrientedDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
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
              new PIDController(
                  DriveConstants.turnKp, DriveConstants.turnKi, DriveConstants.turnKd);
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

          double joyStickOmega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

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
              fieldOrientedSpeeds.left * maxSpeedMetersPerSec,
              fieldOrientedSpeeds.right * maxSpeedMetersPerSec);
        },
        drive);
  }

  /** Measures the velocity feedforward constants for the drive. */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
              timer.restart();
            }),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runOpenLoop(voltage, voltage);
                  velocitySamples.add(drive.getCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }
}
