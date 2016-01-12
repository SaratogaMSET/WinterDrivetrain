
package org.usfirst.frc.team649.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team649.robot.subsystems.DrivetrainSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	 /**
    * This function is run when the robot is first started up and should be
    * used for any initialization code.
    */
	DoubleSolenoid left, right; //grabberPiston2;
	Joystick driverRightJoystick;
	Joystick driverLeftJoystick;
	boolean shift;
	
	DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
	
   public void robotInit() {
   	driverLeftJoystick = new Joystick(0);
   	driverRightJoystick = new Joystick(1);
   	
   	
   	
   	left = new DoubleSolenoid(6, 7);
   	right = new DoubleSolenoid(4,5);
   }

   /**
    * This function is called periodically during autonomous
    */
   public void autonomousPeriodic() {

   }

   /**
    * This function is called periodically during operator control
    */
   public void teleopPeriodic() {
   	shift = driverRightJoystick.getRawButton(1) || driverLeftJoystick.getRawButton(1);
   	drivetrain.driveFwdRot(getDriveForward(), getDriveRotation());
   	shiftDriveGear(shift);
   	
   	SmartDashboard.putData("Encoder Right", drivetrain.encoder);
   }
   
   /**
    * This function is called periodically during test mode
    */
   public void testPeriodic() {
   
   }
   
   public double getDriveForward() {
       return -driverLeftJoystick.getY();
   }

   public double getDriveRotation() {
       final double turnVal = driverRightJoystick.getX();
       final double sign = turnVal < 0 ? -1 : 1;
       return Math.pow(Math.abs(turnVal), 1.4) * sign;
   }
   
   public void shiftDriveGear(boolean lowSpeed) {
   	//default low
       left.set(lowSpeed ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
       right.set(lowSpeed ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);

   }
}