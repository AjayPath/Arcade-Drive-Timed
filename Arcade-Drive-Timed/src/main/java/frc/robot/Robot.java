// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Below are libraries used for REV and Studica Hardware
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/**
 * Main robot class - handles motor setup, joystick input, and periodic control loops.
 */
public class Robot extends TimedRobot {

  // Drivetrain Motors
  private SparkMax leftMotor = new SparkMax(1, MotorType.kBrushless);
  private SparkMax rightMotor = new SparkMax(2, MotorType.kBrushless);

  // Closed Loop Controllers
  private SparkClosedLoopController leftController = leftMotor.getClosedLoopController();
  private SparkClosedLoopController rightController = rightMotor.getClosedLoopController();

  // Encoders
  private RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private RelativeEncoder rightEncoder = rightMotor.getEncoder();

  // Joystick to control robot teleop driving
  private Joystick driveController = new Joystick(0);

  // Variable to track autonomous start time
  private double startTime;

  public Robot() {
    // Configure motors using pre-defined configs
    // This sets up PID, current limits, idle mode, etc.
    leftMotor.configure(Configs.Drivetrain.leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(Configs.Drivetrain.rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Optionally invert motors here if wiring requires it (example: rightMotor.setInverted(true))
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    /**
     * Upon intialization, we can capture the start time
     */

    startTime = Timer.getFPGATimestamp();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    /**
     * This varaible will store the current time
     * Keep in mind autonomous is only 15 seconds
     */
    double time = Timer.getFPGATimestamp();

    /**
     * We can get an elapsed time varaiable using the simple equation below
     */

    double elapsedTime = time - startTime;

    /**
     * We can now create a drive command based on time intervals
     */

    if (elapsedTime < 3) {
      leftMotor.set(0.5);
      rightMotor.set(-0.5);
    }
    else {
      leftMotor.set(0);
      rightMotor.set(0);
    }

  }
  

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    /**
     * The code below lets us constantly read the joystick value during teleop
     * We will use one axis for driving the speed and the other for turning
     */

    double speed = -driveController.getRawAxis(1) * 0.6;  // Slow down the drive speed by 60%
    double turn = driveController.getRawAxis(4) * 0.3;     // Slow down the turning by 30%

    /**
     * Below are the calculations to determine the left and right speeds of the drivetrain
     */

    double leftSpeed = speed + turn;
    double rightSpeed = speed - turn;

    // Convert joystick % speed (-1 to 1) to RPM or native velocity units
    double maxRPM = 5700; // Neo max RPM
    double leftVelocity = leftSpeed * maxRPM;
    double rightVelocity = rightSpeed * maxRPM;

    /**
     * We want the motor to be run at a certain speed during teleop
     */

    leftMotor.setControl(new VelocityVoltage(leftVelocity));
rightMotor.setControl(new VelocityVoltage(rightVelocity));

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

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
