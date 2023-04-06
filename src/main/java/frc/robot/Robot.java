/* 
 Copyright (c) FIRST and other WPILib contributors.
 Open Source Software; you can modify and/or share it under the terms of
 the WPILib BSD license file in the root directory of this project.

 Code written in Spring 2023 by Hector Corpus(Jr), Rafael Cavazos(Comp Sci teacher), 
 and Aiden Iao(Jr). Questions should be direct to these individuals, if they are still around.

 Special thanks to Team 8019 Patriot Engineering and Team 2583 Orange Dynamites
 for their help with our coding.

 This is a Swerve Drive Robot that uses MK4i swerve module wheels for navigation.

 TO DO: MODIFY THE CODE SO THAT AUTONOMOUS IS RUN THROUGH COMMANDS INSTAED OF STRAIGHT 
 FROM THIS FILE.

 Online imports
 https://software-metadata.revrobotics.com/REVLib-2023.json
 https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix5-frc2023-latest.json
 https://raw.githubusercontent.com/SwerveDriveSpecialties/swerve-lib/master/SdsSwerveLib.json
*/
package frc.robot;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.*;
import javax.lang.model.util.ElementScanner14;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  //Creates a motor that we will used as a door for our robot. 
  private CANSparkMax doorMotor = new CANSparkMax(9, MotorType.kBrushless);

  //Creates a encoder that we can assign to a motor
  private RelativeEncoder m_encoder;

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  // We created a timer to use for autonomous
  Timer timer;

  // This is to initialize the port (CS0) on the RoboRIO 1 that we connect the Gyro to
  private static final SPI.Port kGyroPort = SPI.Port.kOnboardCS0;

  // This initializes the Gyro we are using on the robot, and we are setting the channel it is on
  // to be the CS0 port on the RoboRIO
  ADXRS450_Gyro Gyro = new ADXRS450_Gyro(kGyroPort);

  // This string is going to hold the choice for our autonomous commands we choose at startup
  String mode;

  // This sendable chooser is what allows us to decide what autonomous action we want to take.
  // It returns our choice as a String, and we will compare the String to decide what actions
  // our robot will take.
  public static SendableChooser<String> sendableChooser;

  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    //Creates an Relative Encoder that will be used to get the rotational position of the motor.
    m_encoder = doorMotor.getEncoder();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // The timer has been instatiated.
    timer = new Timer();

    // The gyro has been been calibrated to register it as 0 degrees. It takes about 5 seconds to calibrate
    // Keep this in mind when you are trying to use it in autonomous and want to calibrate it in there.
    Gyro.calibrate();

    // This creates a video stream from our camera to driver station. Just be sure to click 
    // "View" -> "Add" -> "CameraServer Stream Viewer" in Smart Dash
    UsbCamera cam0 =CameraServer.startAutomaticCapture(0);
    UsbCamera cam1 =CameraServer.startAutomaticCapture(1);
  

    /* 
     Creates the sendableChooser with a default option and extra options
     Default Option (getOut)to score by moving forward and reversing
     so that we drop a cube into the score area. We will then keep reversing until we are out of the 
     community area.
     Add Option (balance) will score the same way as above, but then will attempt to balance by
     moving back for a set amount of time, then turning on the Gryo and moving autonomously until 
     the Gyro measures at 0 again, at which point it will be balancing on the Charge Station.
    */
    sendableChooser = new SendableChooser<String>();
    sendableChooser.setDefaultOption("Drop and move","drop");
    sendableChooser.addOption("Score and Balance", "balance");
    sendableChooser.addOption("Open Door", "open");
    sendableChooser.addOption("Close Door", "close");
    SmartDashboard.putData("Autonomous", sendableChooser);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // We are reseting the timer to start at 0, then starting it to begin counting so we can move the
    // the robot for set amounts of time.
    timer.reset();
    timer.start();  

    // This stores the option we choose for autonomous into the String we call "mode"
    mode = sendableChooser.getSelected();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    

    
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    if(mode.equalsIgnoreCase("balance")){
      System.out.println(Math.round(Gyro.getAngle()));
      if(timer.get() < 1){
        openDoor();
      }
      else if(timer.get() > 1 && timer.get() < 2){
        closeDoor();
      }
      else if(timer.get() > 2 && timer.get() < 4){
        m_robotContainer.autoBack();
      }
      else if(timer.get() > 4){
        if(Gyro.getAngle() < -5){
          m_robotContainer.autoBackSlow();
        }
        else if(Gyro.getAngle() > 5){
          m_robotContainer.autoMoveSlow();
        }
        else{
          m_robotContainer.autoStop();
        }
      }
    }

    else if(mode.equalsIgnoreCase("Drop")){
      if(timer.get() < 1){
        openDoor();
      }
      else if(timer.get() < 2){
       closeDoor();
      }
      else if(timer.get() < 6.5){
        m_robotContainer.autoBack();
      }
    }

    else if(mode.equalsIgnoreCase("open")){
      openDoor();
    }

    else if(mode.equalsIgnoreCase("close")){
      closeDoor();
    }
    
   }
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  //Opens the door motor if the encoder is under a certain position about 1/4 of the gear rotation.
  public void openDoor(){
    moveDoorBack();
    if(m_encoder.getPosition() < 0.25){
      //The speed at which the motor will turn the door.
      doorMotor.set(0.05);
    }
    else{                       
      doorMotor.stopMotor();
    }
  }
  //Closes the door motor if the encoder is above the encoder position 0, which means its closed.
  public void closeDoor(){
    moveDoorBack();
    if(m_encoder.getPosition() > 0){
      //The speed at which the motor will turn the door.
      doorMotor.set(-0.05);
    }
    else{
      doorMotor.stopMotor();
    }
  }

  //Moves the motor to the position it should be if it get out of the bounds.
  public void moveDoorBack(){
    if(m_encoder.getPosition() < 0 || m_encoder.getPosition() > 0.25){
      if(m_encoder.getPosition() < 0){
        doorMotor.set(0.05);
      }
      else if(m_encoder.getPosition() > 0.25){
        doorMotor.set(-0.05);
      }
      else{
        doorMotor.stopMotor();
      }
    }
  }

  @Override
  public void teleopPeriodic() {
    //Keeps the door in the encoder range of 0 to 0.25, 
    //Bacically door no hit other things, stay in safe spot
    moveDoorBack();

    //Open Door with right bumper
    if(m_robotContainer.controller.getRightBumper() && m_encoder.getPosition() < 0.25){
      doorMotor.set(0.05);
    }
    
    //Close Door with left bumper
    else if(m_robotContainer.controller.getLeftBumper() && m_encoder.getPosition() > 0){
      doorMotor.set(-0.05);
    }

    else{
      doorMotor.stopMotor();
    }

    if(m_robotContainer.controller.getAButtonPressed())
    {
      System.out.println("Encoder Value: " + m_encoder.getPosition());
    }

    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
