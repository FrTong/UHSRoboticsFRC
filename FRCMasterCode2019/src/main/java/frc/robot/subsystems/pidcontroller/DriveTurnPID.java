/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.pidcontroller;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.Constant;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DriveTurnPID extends PIDSubsystem {
  /**
   * Does a pivot turn
   */
  public DriveTurnPID() {
    // Intert a subsystem name and PID values here
    super("DriveTurnPID", 0.2, 0, 0);
    setAbsoluteTolerance(1.0); //Degree of error
    getPIDController().setContinuous(true);
    setInputRange(-360, 360);
    setOutputRange(-Constant.PID_DRIVE_OUTPUT, Constant.PID_DRIVE_OUTPUT);
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  protected double returnPIDInput() {
    return RobotMap.gyro.getAngle();
  }

  @Override
  protected void usePIDOutput(double output) {
    Robot.driveSubsystem.drive(output, -output);
  }
}
