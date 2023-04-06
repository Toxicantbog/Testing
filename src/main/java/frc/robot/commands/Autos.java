// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DrivetrainSubsystem;

public final class Autos {

  // Declare and initialize the necessary variables
  //private final static double kDriveSpeed = 0.5; // Adjust this value to change the robot's speed (this should be half speed)
  //private final static double kDriveTime = 1.0;// Adjust this value to change the duration of movement (this should be 1 second)
  //private final static DrivetrainSubsystem driveTrain = new DrivetrainSubsystem();// Swerve Drive System to move
  //private static ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.5, 0.0, 0.0);

  /** Example static factory for an autonomous command. */
  
  public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  //public static void exampleAuto(ExampleSubsystem subsystem) {
  //  double startTime = Timer.getFPGATimestamp(); // Get the current time
  //  while(Timer.getFPGATimestamp() - startTime < kDriveTime) {
  //    driveTrain.drive(chassisSpeeds); 
  //  }
  //}
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}

