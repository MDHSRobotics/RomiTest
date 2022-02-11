// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants;

public class ArcadeDrive extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final Supplier<Double> m_xaxisSpeedSupplier;
  private final Supplier<Double> m_zaxisRotateSupplier;
  private final SlewRateLimiter turningLimiter;
  private final SlewRateLimiter drivingLimiter;

  /**
   * Creates a new ArcadeDrive. This command will drive your robot according to the speed supplier
   * lambdas. This command does not terminate.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   * @param xaxisSpeedSupplier Lambda supplier of forward/backward speed
   * @param zaxisRotateSupplier Lambda supplier of rotational speed
   */
  public ArcadeDrive(
      Drivetrain drivetrain,
      Supplier<Double> xaxisSpeedSupplier,
      Supplier<Double> zaxisRotateSupplier) {
    m_drivetrain = drivetrain;
    drivingLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    turningLimiter = new SlewRateLimiter(Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    m_xaxisSpeedSupplier = xaxisSpeedSupplier;
    m_zaxisRotateSupplier = zaxisRotateSupplier;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double drivingSpeed = drivingLimiter.calculate(m_xaxisSpeedSupplier.get());
    double turningSpeed = turningLimiter.calculate(m_zaxisRotateSupplier.get());
    m_drivetrain.arcadeDrive(drivingSpeed, turningSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
