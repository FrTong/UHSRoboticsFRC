/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.pidcontroller.DriveStraightPID;

public class DriveStraightByDist extends Command {
  private int dist;
  private int count;
  private int timeout;

  //don't use this
  private DriveStraightByDist(){}
  
  public DriveStraightByDist(double pow, int dist,int timeout) {
    requires(Robot.driveSubsystem);
    initVar(dist, timeout);
    Robot.driveStraightPID = new DriveStraightPID(pow,dist);
    Robot.driveStraightPID.disable();
  }

  public void initVar(int dist, int timeout){
    this.dist = dist;
    this.timeout = timeout;
    count = 0;
  }
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    RobotMap.gyro.reset();
    RobotMap.leftEncoder.reset();
    RobotMap.rightEncoder.reset();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    count++;
    if(count == 5){ //100 ms to reset, need test
      Robot.driveStraightPID.setSetpoint(0); //drive straight
      Robot.driveStraightPID.enable();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(RobotMap.leftEncoder.getDistance() >= dist){
      return true;
    }
    return count >= timeout/0.2;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveStraightPID.disable();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
