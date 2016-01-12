package org.usfirst.frc.team649.robot.subsystems;


import org.usfirst.frc.team649.robot.RobotMap;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DrivetrainSubsystem extends PIDSubsystem {
    
    public Victor [] motors;
    public Encoder encoder;
    public PIDController encoderDriveLeftPID;
    
    
    public DrivetrainSubsystem() {
    	super("Drivetrain", 1.0, 0, 0);
    	motors = new Victor[4];
       	motors[0] = new Victor(0);
       	motors[1] = new Victor(9);
       	motors[2] = new Victor(2);
       	motors[3] = new Victor(3);
       	
    	encoderDriveLeftPID = this.getPIDController();
    	encoderDriveLeftPID.setAbsoluteTolerance(0.5);
    	//encoderDrivePID.setOutputRange(-EncoderBasedDriving.MAX_MOTOR_POWER, EncoderBasedDriving.MAX_MOTOR_POWER);
        encoder = new Encoder(8, 9, true);

        encoder.setDistancePerPulse(11.0);
        
    }
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    

    public void driveFwdRot(double fwd, double rot) {
        double left = fwd + rot, right = fwd - rot;
        double max = Math.max(1, Math.max(Math.abs(left), Math.abs(right)));
        left /= max;
        right /= max;
        rawDrive(left, right);
    }

    public void rawDrive(double left, double right) {
        int i = 0;
        left /= 1.0;//left > 0 ? Math.pow(left, 2)/2 : -Math.pow(left, 2)/2;
        right /= 1.0;//right > 0 ? Math.pow(right, 2)/2 : -Math.pow(right, 2)/2;
        for (; i < motors.length / 2; i++) {
            motors[i].set(left);
        }

        for (; i < motors.length; i++) {
            motors[i].set(-right);
        }
    }
    
    public double getDistance() {
//        int numEncoders = encoders.length;
//        double totalVal = 0;
//        for (int i = 0; i < numEncoders; i++) {
//            totalVal += encoders[i].getDistance();
//        }
//        return totalVal / numEncoders;
        
        return encoder.getDistance();
    }

    
    
    public void resetEncoders() {
        encoder.reset();
    }
    
	protected double returnPIDInput() {
		return this.getDistance();
	}

	protected void usePIDOutput(double output) {
        driveFwdRot(output, 0);
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
//    public PIDController getGyroPIDControler() {
//    	return encoderTurnPID;
//    }
//	@Override
//	public void pidWrite(double output) {
//        
//        driveFwdRot(0, output);
//	}
//	@Override
//	public double pidGet() {
//		return this.encoders[0].getDistance();
//	}
}

