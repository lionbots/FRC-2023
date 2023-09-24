// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
// edu.wpi.first.wpilibj.motorcontrol; 
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;




/*
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  //SparkMax for CANId: 1
  private final CANSparkMax grab1 = new CANSparkMax(1, MotorType.kBrushless);
  //SparkMax for CANId: 2
  private final CANSparkMax grab2 = new CANSparkMax(2, MotorType.kBrushless);
  private final WPI_TalonSRX FLeft = new WPI_TalonSRX(7);
  private final WPI_TalonSRX FRight = new WPI_TalonSRX(3);
  private final WPI_TalonSRX BLeft = new WPI_TalonSRX(8);
  private final WPI_TalonSRX BRight = new WPI_TalonSRX(4);
  private final WPI_TalonSRX arm = new WPI_TalonSRX(5);
  private final MotorControllerGroup leftGroup = new MotorControllerGroup(FLeft, BLeft); 
  private final MotorControllerGroup rightGroup = new MotorControllerGroup(FRight, BRight);
  
  
  DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);

  private final Joystick joystick = new Joystick(0);
  //private final Joystick joystick2 = new Joystick(1);

  private final JoystickButton button1 = new JoystickButton(joystick, 1);
  private final JoystickButton button2 = new JoystickButton(joystick, 2);
  private final JoystickButton button3 = new JoystickButton(joystick, 3);
  private final JoystickButton button4 = new JoystickButton(joystick, 4);
  private final JoystickButton button8 = new JoystickButton(joystick, 8);
  private long time = 0; 
  private double armSp = 0.0;
  private double holdPower = -0.1;

  private boolean armPressed = false;


  @Override
  public void robotInit() {
    
    new Thread(() -> {
       // CameraServer.getInstance();
       UsbCamera camera = CameraServer.startAutomaticCapture(0);
       camera.setResolution(192, 144);
   
       //CameraServer.getInstance();
       UsbCamera camera1 = CameraServer.startAutomaticCapture(1);
       camera1.setResolution(192, 144);
       
       CameraServer.getVideo();
       CvSink cvSink = CameraServer.getVideo();
       //CameraServer.getInstance();
       CvSource outputStream = CameraServer.putVideo("Blur", 640, 480);
       
       
       Mat source = new Mat();
       Mat output = new Mat();
       
       while(!Thread.interrupted()) {
           cvSink.grabFrame(source);
           Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
           outputStream.putFrame(output);
       }
   }).start(); 
   
 
   FLeft.setNeutralMode(NeutralMode.Brake);
    FRight.setNeutralMode(NeutralMode.Brake);
    BLeft.setNeutralMode(NeutralMode.Brake);
    BRight.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    time = System.currentTimeMillis(); 
  }

  @Override
  public void autonomousPeriodic() {
    goBackAuto();
  }

  @Override
  public void teleopInit() {} 

  //---- Controls ----
  @Override
  public void teleopPeriodic() {

    //---- Used to drive Robot ----
    drive.arcadeDrive(joystick.getZ()/1.5, joystick.getY());
     
    /* Testing which IDs the buttons are attached to in order to program correctly in Joystick section
     */

    /*if(button1.get()){
      arm.set(.1);
    
    } else {
      arm.set(0);

    }
    /*if(button2.get()){
      grab1.set(.1);
      grab2.set(.1);
    } else {
      grab1.set(0);
      grab2.set(0);
    }

    if(button3.get()){
      grab1.set(-.1);
      grab2.set(-.1);
    } else {
      grab1.set(0);
      grab2.set(0);
    }
    */

    //---- Grippers ----
    if(joystick.getRawButton(3)){
      // ---- Intake ----
       grab1.set(-0.3);
       grab2.set(0.3);
       
    } else   if (joystick.getRawButton(4)){
      // ---- Outake ----
      grab1.set(0.6);
      grab2.set(-0.6);
    } else {
      grab1.set(-0.02);
      grab2.set(0.02);
    }

    //---- Arm ---- 
      // ---- WIP -----
    if (joystick.getRawButton(10)) {
      if (armSp < 0.3) {
        armSp += 0.005;
        arm.set(armSp);
  
      } else {
        arm.set(armSp);
      }

    } else if (joystick.getRawButton(8)) {
      if (armSp > -0.3) {
        armSp -= 0.005;
        arm.set(armSp);
      } else {
        arm.set(armSp);
      }
    } else if (joystick.getRawButton(9)) {
      if (armSp > holdPower) {
        armSp -= 0.005;
        arm.set(armSp);
      } else if (armSp < holdPower) {
        armSp += 0.005;
        arm.set(armSp);
      } 
    }else {
      if (armSp > 0) {
        armSp -= 0.005;
        arm.set(armSp);
      } else if (armSp < 0) {
        armSp += 0.005;
        arm.set(armSp);
      } 
      //armSp = 0.0;
      //arm.set(armSp);
    } 

      // --- Old working code ----
    /*if(joystick.getRawButton(1)){
      arm.set(0.3);
      armPressed = true;
      // System.out.println("here");
    } else if(joystick.getRawButton(2)){
      arm.set(-0.3);
      armPressed = true;
      // System.out.println("here");
    } else {
      arm.set(0.0);
      armPressed = false;
    } */


    
    /*if (button1.get()){
       System.out.println("button 1");
    } else if (button2.get()){
      System.out.println("button 2");
    } else if (button3.get()){
      System.out.println("button 3");
    } */
  }

   

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  //---- Auto (?) ---- 
  public void goBackAuto() {
    grab1.set(0.6);
    grab2.set(-0.6);
    
    if (System.currentTimeMillis() - time < 2000) {
    drive.tankDrive(0.5, -0.5);
    } else {
      drive.tankDrive(0, 0);
    }
  }
}