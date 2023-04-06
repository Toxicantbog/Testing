// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DrivetrainSubsystem driveTrain = new DrivetrainSubsystem();
  // Creates an xbox controller that will be read up on plugging it into a USB Port (USB C works, too)
  public final XboxController controller = new XboxController(0);
  // 1 moves the robot, 2 stops the robot. They are used in the autoStart() and autoStop() methods.
  // 3 moves the robot backwards (reverse), and 4 rotates the robot 180 degrees.
  private ChassisSpeeds chassisSpeeds1 = new ChassisSpeeds(1.0, 0.0, 0.0);
  private ChassisSpeeds chassisSpeeds2 = new ChassisSpeeds(0.0, 0.0, 0.0);
  private ChassisSpeeds chassisSpeeds3 = new ChassisSpeeds(-1.0, 0.0, 0.0);
  private ChassisSpeeds chassisSpeeds4 = new ChassisSpeeds(0, 0, 3.14);
  private ChassisSpeeds chassisSpeeds5 = new ChassisSpeeds(0.25, 0.0, 0.0);
  private ChassisSpeeds chassisSpeeds6 = new ChassisSpeeds(-0.25, 0.0, 0.0);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings


    // We multiply by 0.80 to slow down the speed of the wheels. Mess with the number
    // to get the speed we want. 
    // There was no previous value that we multiplied by, if you want to set
    // it back to stock settings.
    driveTrain.setDefaultCommand(new DriveCommand(
      driveTrain,
      () -> -modifyAxis(controller.getLeftY() * 0.80), // Axis are flipped here on purpose
      () -> -modifyAxis(controller.getLeftX() * 0.80),
      () -> -modifyAxis(controller.getRightX() * 0.80)
    ));

    configureBindings();
  }

  public static double deadband(double value, double deadband) {
    if(Math.abs(value) > deadband) {
      if(value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  // Change the value of deadband in the parenthesis to increase the deadzone
  // A number between 0 and 1, bigger number should mean bigger deadzone
  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.1);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
      //  .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  // This method is called by autonomousPeriodic(). It sets the chassisSpeed to half of max,
  // and hopefully moves it forward. It should move for about 1 second, or close to it.
  public void autoMove(){
    driveTrain.drive(chassisSpeeds1);
  }

  // This method is called by autonomousPeriodic(). It sets the chassisSpeed to 0,
  // and hopefully stops the robot.
  public void autoStop(){
    driveTrain.drive(chassisSpeeds2);
  }

  //This method ia called by the autonomousPeriodic(). It sets the chassisSpeed to -1.0 for vxMeterPerSecond
  //makes the robot move backwards
  public void autoBack(){
    driveTrain.drive(chassisSpeeds3);
  }

  // This method is called by autonomousPeriodic(). It rotates the robot 180 degrees in autonomous by
  // changing the last parameter in chassisSpeeds4
  public void autoRotate(){
    driveTrain.drive(chassisSpeeds4);
  }

  // This method moves the robot forward slowly when we are trying to balance on the charge station
  public void autoMoveSlow(){
    driveTrain.drive(chassisSpeeds5);
  }

    // This method moves the robot backwards slowly when we are trying to balance on the charge station

  public void autoBackSlow(){
    driveTrain.drive(chassisSpeeds6);
  }
}
