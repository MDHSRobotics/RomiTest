// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnDegreesWithGyro extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_targetAngleDegrees;
  private final double m_speed;
  private double m_gyroStartAngle;

  /**
   * Creates a new TurnDegreesWithGyro. This command will turn your robot for a desired rotation (in
   * degrees) and rotational speed; then stop based on the change in gryo reading.
   *
   * @param speed The speed which the robot will drive. Negative is in reverse.
   * @param m_targeAngleDegrees Degrees to turn. Leverages gyro to compare distance.
   * @param drive The drive subsystem on which this command will run
   */
  public TurnDegreesWithGyro(double speed, double targetAngleDegrees, Drivetrain drive) {
    m_targetAngleDegrees = targetAngleDegrees;
    m_speed = speed;
    m_drive = drive;
    m_gyroStartAngle = 0.0;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set motors to stop, read encoder values for starting point
    m_drive.arcadeDrive(0, 0);
    m_gyroStartAngle = m_drive.getGyroAngleZ();

    System.out.format("Initial gyro angle is %.2f\n", m_gyroStartAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO This should really use some kind of PID controller because with a constant
    // power, the rotation is guaranteed to overshoot the target angle by several degrees
    m_drive.arcadeDrive(0, m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);

    double finalGyroAngle = m_drive.getGyroAngleZ();
    double overRun = finalGyroAngle - m_gyroStartAngle - m_targetAngleDegrees;
    System.out.format("Final gyro reading = %.2f, which is an overrun of %.2f\n", finalGyroAngle, overRun);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Get the current gyro bearing and compare it to the start angle
    double currentGyroAngle = m_drive.getGyroAngleZ();
    double deltaAngle = currentGyroAngle - m_gyroStartAngle;

    System.out.format("Current angle = %.2f; target angle = %.2f; delta angle = %.2f\n", currentGyroAngle, m_gyroStartAngle, deltaAngle);

    // TODO Needs to handle a) negative angles, b) turning in negative direction; c) angles that flip due to modulus 360
    boolean finished = deltaAngle >= m_targetAngleDegrees;

    return finished;
  }
}
