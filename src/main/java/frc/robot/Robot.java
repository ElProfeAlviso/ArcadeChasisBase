// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  private final VictorSP leftMotors = new VictorSP(0);
  private final VictorSP rightMotors = new VictorSP(1);

  private final DifferentialDrive robotDrive = new DifferentialDrive(leftMotors, rightMotors);

  private final PS4Controller joystick = new PS4Controller(0);

  private final Timer timer = new Timer();

  public Robot() {
    SendableRegistry.addChild(robotDrive, leftMotors);
    SendableRegistry.addChild(robotDrive, rightMotors);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightMotors.setInverted(true);

    SmartDashboard.putNumber("Chasis Speed", 0.5);

    // Inicia la captura automática de la cámara USB
        UsbCamera camera = CameraServer.startAutomaticCapture("USB Camera 0", 0);
        // Establece la resolución y la velocidad de cuadros
        camera.setResolution(320, 240);
        camera.setFPS(15);
        // Configuración adicional si es necesaria
        camera.setBrightness(50); // Ajustar el brillo
        camera.setExposureManual(50); // Ajustar la exposición
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (timer.get() < 2.0) {
      // Drive forwards half speed, make sure to turn input squaring off
      robotDrive.arcadeDrive(0.5, 0.0, false);
    } else {
      robotDrive.stopMotor(); // stop robot
    }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {

   double chasisSpeed = SmartDashboard.getNumber("Chasis Speed", 0);


    robotDrive.arcadeDrive(-joystick.getLeftY()*chasisSpeed, -joystick.getRightX()*chasisSpeed);

    SmartDashboard.putNumber("Motor Speed Y", joystick.getLeftY());
    SmartDashboard.putNumber("Motor Speed X", joystick.getRightX());

    
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
  

  //Dashboard Outs

  


  







}
