
package org.usfirst.frc.team649.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team649.robot.commands.DrivePIDLeft;
import org.usfirst.frc.team649.robot.commandgroups.ShootTheShooter;
import org.usfirst.frc.team649.robot.commands.DriveForwardRotate;
import org.usfirst.frc.team649.robot.commands.DrivePIDRight;
import org.usfirst.frc.team649.robot.commands.MatchAutoDrive;
import org.usfirst.frc.team649.robot.subsystems.IntakeSubsystem;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.AutonomousSequences;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.DrivetrainSubsystem;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.LeftDTPID;
import org.usfirst.frc.team649.robot.subsystems.drivetrain.RightDTPID;
import org.usfirst.frc.team649.robot.subsystems.shooter.ShooterSubsystem;

import java.io.DataOutputStream;
import java.io.File;
import java.io.FileWriter;
import java.io.OutputStream;
import java.util.ArrayList;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	public static double DEAD_ZONE_TOLERANCE = 0.025;
	public static double RPM_TOLERANCE = 100;
	public static ArrayList <ArrayList<Double>> log;
	
	public static Timer timer;
	
	public static FileWriter writer; 
	
	 /**
	  * 
    * This function is run when the robot is first started up and should be
    * used for any initialization code.
    */
	
	//Joysticks
	Joystick driverRightJoystick;
	Joystick driverLeftJoystick;
	Joystick drivetrainPIDTuningJoystick; 
	Joystick operatorJoystick;
	Joystick shooterPIDTuningJoystick;
	
	//Drivetrain
	boolean shift;
	DoubleSolenoid driveSol; //grabberPiston2;
	//PID
	double drivetrain_p_value;
	double drivetrain_i_value;
	double drivetrain_d_value;
	//subsystems
	public static DrivetrainSubsystem drivetrain;
	public static LeftDTPID leftDT;
	public static RightDTPID rightDT;
	
	//prev states for joysticks
	private boolean prevStateLeft6Button;
	private boolean prevStateLeft7Button;
	private boolean prevStateRight11Button;
	private boolean prevStateRight10Button;
	private boolean prevStateOperatorTrigger;
	//
	private boolean prevState3Button;
	private boolean prevState1Button;
	private boolean prevState2Button;
	
	public static double INCREMENT = 0.001;
	public static boolean isPIDActiveLeft = false;
	public static boolean isPIDActiveRight = false;
	
	public static boolean canShoot = false;
	
	
	double powerToSet;
	
	
	public IntakeSubsystem intake;
	public static ShooterSubsystem shooter;
	
	
	
   public void robotInit() {
		drivetrain = new DrivetrainSubsystem();
		shooter = new ShooterSubsystem();
		intake = new IntakeSubsystem();
		
		leftDT = new LeftDTPID();
		rightDT = new RightDTPID();
		
	   	driverRightJoystick = new Joystick(0);
	   	driverLeftJoystick = new Joystick(1);
	   	operatorJoystick = new Joystick(2);
	   	drivetrainPIDTuningJoystick = new Joystick(3);
	   	shooterPIDTuningJoystick = new Joystick(4);
	   	
	   	drivetrain_p_value = leftDT.getPIDController().getP();
	   	drivetrain_i_value = leftDT.getPIDController().getI();
	   	drivetrain_d_value = leftDT.getPIDController().getD();
	   	
	   	prevStateLeft6Button = false;
	    prevStateLeft7Button = false;
	    prevStateRight11Button = false;
	    prevStateRight10Button = false;
	    prevState3Button = false;
	    prevState1Button = false;
	    prevState2Button = false;
	    
	    prevStateOperatorTrigger = false;
	   	
	   	driveSol = new DoubleSolenoid(6, 7);
   	
   		log = new ArrayList< ArrayList<Double>>();
   }
   
   public void disabledPeriodic() {
		Scheduler.getInstance().run();
		
		new DrivePIDLeft(0).start();
		new DrivePIDRight(0).start();
	}
   
   public void autonomousInit(){
	   Command testAuto = new MatchAutoDrive(AutonomousSequences.p_test);
	   testAuto.start();
	   
	   
   }
   
   /**
    * This function is called periodically during autonomous
    */
   public void autonomousPeriodic() {
	   Scheduler.getInstance().run();
	   
   }

   public void teleopInit(){
	   drivetrain.resetEncoders();
	   
	   log = new ArrayList< ArrayList<Double>>();
	   
	   timer = new Timer();
	   timer.reset();
	   timer.start();
	   
	   new DrivePIDLeft(0).start();
	   new DrivePIDRight(0).start();
	   
	   try{
		   File f = new File("output.txt");
		   writer = new FileWriter(f);
		   
		   writer.write("CRAP TO REMEMBER FOR LATER");
	   }
	   catch (Exception e){
		   System.out.println(e.getMessage());
	   }
   }
   /**
    * This function is called periodically during operator control
    */
   public void teleopPeriodic() {
	   Scheduler.getInstance().run();
	   
	   
	   //DT
	   shift = driverRightJoystick.getRawButton(1) || driverLeftJoystick.getRawButton(1);
	   if (!(isPIDActiveLeft || isPIDActiveRight) ){
		   new DriveForwardRotate(getDriveForward(), getDriveRotation()).start();
	   }
	   shiftDriveGear(shift);
	   
	   //LOGGING
	   try{
		   
		   writer.write(timer.get() + ", " + drivetrain.motors[0].get() + ", " + drivetrain.motors[1].get());
		   //writer.close();
	   }
	   catch (Exception e){
		   System.out.println(e.getMessage());
	   }
	   
	   ArrayList<Double> temp = new ArrayList<Double>(5);
	   temp.add(timer.get());
	   temp.add(drivetrain.motors[0].get());
	   temp.add(drivetrain.motors[1].get());
	   temp.add(drivetrain.getDistanceDTLeft());
	   temp.add(drivetrain.getDistanceDTRight());
	   
	   log.add(temp);
	   
	   //shooter
	   //powerToSet = correctForDeadZone(-operatorJoystick.getThrottle(), 0.2); //(-operatorJoystick.getThrottle() + 1.0) / 2.0);
	   
	   //bang bang when holding thumb trigger
	   if (operatorJoystick.getRawButton(2)){
		   
		   //1
		   if (shooter.getRPMEinstein1() < ShooterSubsystem.TARGET_SHOOT_SPEED){
			   shooter.rollers[0].set(0.55); //0.55
		   }
		   else {
			   shooter.rollers[0].set(0.3); //0.3
		   }
		   
		   //2
		   if (shooter.getRPMEinstein2() < ShooterSubsystem.TARGET_SHOOT_SPEED){
			   shooter.rollers[1].set(-0.55);
		   }
		   else {
			   shooter.rollers[1].set(-0.3);
		   }
		   
		   if (Math.abs(shooter.getRPMEinstein1() - ShooterSubsystem.TARGET_SHOOT_SPEED) < RPM_TOLERANCE
				   && Math.abs(shooter.getRPMEinstein2() - ShooterSubsystem.TARGET_SHOOT_SPEED) < RPM_TOLERANCE){
			   canShoot = true;
			   if (operatorJoystick.getRawButton(1) && !prevStateOperatorTrigger){
				   new ShootTheShooter().start();
			   }
		   }
		   else{
			   canShoot = false;
		   }
		   
//		   if (operatorJoystick.getRawButton(1)){
//			   shooter.setRollerSpeed(powerToSet);
//		   } else {
//			   shooter.setRollerSpeed(0);
//		   }
	   }
	   else{
		   shooter.setRollerSpeed(0);
	   }
	   
	   //TEMP
//	   if (operatorJoystick.getRawButton(1) && !prevStateOperatorTrigger){
//			   new ShootTheShooter().start();
//	   }
	  
	   
	   //SmartDashboard.putNumber("Setpower" ,powerToSet);
	   //intakes
	   if (operatorJoystick.getRawButton(11)){
		  intake.setRollerSpeed(IntakeSubsystem.INTAKE_SPEED);
	   }
	   else if (operatorJoystick.getRawButton(12)){
		  intake.setRollerSpeed(IntakeSubsystem.PURGE_SPEED);
	   }
	   else{
		  intake.setRollerSpeed(0);
	   }
	   
	   SmartDashboard.putBoolean("Can Shoot", canShoot);
	   SmartDashboard.putNumber("Throttle", -operatorJoystick.getThrottle());
	   SmartDashboard.putBoolean("Trigger", operatorJoystick.getRawButton(1));
	   
	   SmartDashboard.putBoolean("Can shoot Einstein 1", shooter.getRPMEinstein1() < ShooterSubsystem.TARGET_SHOOT_SPEED);
	   SmartDashboard.putBoolean("Can shoot Einstein 2", shooter.getRPMEinstein2() < ShooterSubsystem.TARGET_SHOOT_SPEED);
	   
	   //auto pid tuning
	   Command d_l, d_l_2, d_r, d_r_2;
	   
	 
	   
	   //left side (
	   if (driverLeftJoystick.getRawButton(6) && !prevStateLeft6Button){
		   d_l = new DrivePIDLeft(20);
		   d_l.start();
	   }
	   else if (driverLeftJoystick.getRawButton(7) && !prevStateLeft7Button){
		   d_l_2 = new DrivePIDLeft(-20);
		   d_l_2.start();
	   }
	   
	   //right side
	   if (driverRightJoystick.getRawButton(11) && !prevStateRight11Button){
		   d_r = new DrivePIDRight(20);
		   d_r.start();
	   }
	   else if (driverRightJoystick.getRawButton(10) && !prevStateRight10Button){
		   d_r_2 = new DrivePIDRight(-20);
		   d_r_2.start();
	   }
	   
	   //set PID
	   //on GAMEPAD: P:X-button, I:A-button, D:Y-button
	   //			DECREASE^^ by simultaneously holding LB button
	   if(drivetrainPIDTuningJoystick.getRawButton(3) && !prevState3Button){
		   drivetrain_p_value+= (drivetrainPIDTuningJoystick.getRawButton(5)? -INCREMENT : INCREMENT);
	   }
	   if(drivetrainPIDTuningJoystick.getRawButton(1) && !prevState1Button){
		   drivetrain_i_value+= (drivetrainPIDTuningJoystick.getRawButton(5)? -INCREMENT : INCREMENT);
	   }
	   if(drivetrainPIDTuningJoystick.getRawButton(2) && !prevState2Button){
		   drivetrain_d_value+= (drivetrainPIDTuningJoystick.getRawButton(5)? -0.0001 : 0.0001);
	   }
		
	   //check to make sure none are under 0
	   if (drivetrain_p_value < 0) drivetrain_p_value=0;
	   if (drivetrain_i_value < 0) drivetrain_i_value=0;
	   if (drivetrain_d_value < 0) drivetrain_d_value=0;
	   
	   //update PID
	   leftDT.getPIDController().setPID(drivetrain_p_value, drivetrain_i_value, drivetrain_d_value);
	   rightDT.getPIDController().setPID(drivetrain_p_value, drivetrain_i_value, drivetrain_d_value);
	   
	   //for drive pid commands
	   prevStateLeft6Button = driverLeftJoystick.getRawButton(1);
	   prevStateLeft7Button = driverLeftJoystick.getRawButton(2);
	   prevStateRight11Button = driverRightJoystick.getRawButton(11);
	   prevStateRight10Button = driverRightJoystick.getRawButton(10);
	   //PID tuning
	   prevState3Button = drivetrainPIDTuningJoystick.getRawButton(3);
	   prevState1Button = drivetrainPIDTuningJoystick.getRawButton(1);
	   prevState2Button = drivetrainPIDTuningJoystick.getRawButton(2);
	   //shooter trigger
	   prevStateOperatorTrigger = operatorJoystick.getRawButton(1);
	   
	   SmartDashboard.putNumber("P", drivetrain_p_value); 
	   SmartDashboard.putNumber("I", drivetrain_i_value);
	   SmartDashboard.putNumber("D", drivetrain_d_value);
	   SmartDashboard.putData("Encoder Left", drivetrain.encoders[0]);
	   SmartDashboard.putData("Encoder Right", drivetrain.encoders[1]);
	   SmartDashboard.putBoolean("Is PID Active?", isPIDActiveLeft);
	   SmartDashboard.putNumber("Einstein 1 RPM", 60.0/shooter.photoSensor1.getPeriod());
	   SmartDashboard.putNumber("Einstein 2 RPM", 60.0/shooter.photoSensor2.getPeriod());
	   SmartDashboard.putNumber("Einstein Difference RPM", Math.abs(60.0/shooter.photoSensor2.getPeriod() - 60.0/shooter.photoSensor1.getPeriod()));
	   
	   //System.out.println("Log working, size: " + log.size());
	   //SmartDashboard.putNumber("Abs Encoder: getAverageValue()", shooter.absEncoder.getAverageValue());
   }
   
   public void disabledInit(){
	   new DrivePIDLeft(0).start();
	   new DrivePIDRight(0).start();
	   
	   try{
		   writer.close();
	   }
	   catch(Exception e){
		   
	   }
	   
	   //LOG printer
	   System.out.println("STARTING LOG: Time, MotorLeft, MotorRight ");
	   for (int i = 0; i < log.size(); i++){
		   ArrayList<Double> d = log.get(i);
		   System.out.println("BEGINNING_TAG " + d.get(0) + ", " + d.get(1) + ", " + d.get(2) + ", " + d.get(3) + ", " + d.get(4) + " ENDING_TAG");
	   }
   }
   
   /**
    * This function is called periodically during test mode
    */
   public void testPeriodic() {
	   Scheduler.getInstance().run();
	  
   }
   
   
   // *******************************ADDED FUNCTIONS******************************//
   
   public double correctForDeadZone(double joyVal, double minTolerance){
	   return Math.abs(joyVal) >= minTolerance ? joyVal : 0;
   }
   
   //default tolerance
   public double correctForDeadZone(double joyVal){
	   return Math.abs(joyVal) >= DEAD_ZONE_TOLERANCE ? joyVal : 0;
   }
   
   public double getDriveForward() {
       return correctForDeadZone(-driverLeftJoystick.getY());
   }

   public double getDriveRotation() {
       final double turnVal = correctForDeadZone(driverRightJoystick.getX());
       final double sign = turnVal < 0 ? -1 : 1;
       return Math.pow(Math.abs(turnVal), 1.4) * sign;
   }
   
   public void shiftDriveGear(boolean lowSpeed) {
   	//default low
       driveSol.set(lowSpeed ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
       
       //for future implementation of shifting DT with solenoids and correcting encoders accordingly
//       if (!lowSpeed && prevSolState){
//    	   encoder.setDistancePerPulse(asfnsfna,s);
//       }

   }
}