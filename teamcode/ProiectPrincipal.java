package org.firstinspires.ftc.teamcode;
 
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.hardware.CRServo;
// import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

import java.util.concurrent.TimeUnit;

@TeleOp
public class ProiectPrincipal extends LinearOpMode {
	// double poz_servo1=0.5;
	double MIN_POSITION = 0;
	double MAX_POSITION = 1;
	boolean gheara_toggle=true;
	boolean am_con=false;
	// long t; // gheara
	// boolean okg=true; // pot intra sa verific daca e apasat?
 
	@Override
	public void runOpMode() throws InterruptedException {
		telemetry.addData("Mode", "waiting");
		telemetry.update();
 
		// --- lift init, vezi ../subsystems/Lift.java
		// uncomment
		// Lift lift = new Lift();
		// ---

		// --- motoare sasiu init
		DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
		DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
		DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
		DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
		// ---
 
		// --- continuous servo init
		// uncomment astea:
		// CRServo servo1 = hardwareMap.crservo.get("servoRotatie");
		// CRServo servo2 = hardwareMap.crservo.get("gheara1");
		// CRServo servo3 = hardwareMap.crservo.get("gheara2");
		
		// ---
 
		// --- motoare sasiu directie, pe zero input/power = stationar
 
		// Reverse the right side motors
		// Reverse left motors if you are using NeveRests
		// uncomment:
		motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
		motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
 
		motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		// ---
 
		waitForStart();
		
		// uncomment:
		// lift.init(hardwareMap);

		if (isStopRequested()) return;
 
		while (opModeIsActive()) {
			// --- servo rotatie brat
			// uncomment
			// double y2 = -gamepad2.left_stick_x * 0.2; // Remember, this is reversed!
			// servo1.setPower(y2); // sper ca e de la -1 la 1
			// ---
 
			// --- reset gheara, INLOCUIESTE while(System.currentTimeMillis() < t)...
			// if(System.currentTimeMillis()>t) {
			//	 if(gheara_toggle==true){
			//		 servo2.setPower(-0.01);
			//		 servo3.setPower(0.01);
			//		 gheara_toggle=false;
			//	 }
			//	 else {
			//		 servo2.setPower(0.0);
			//		 servo3.setPower(0.0);
			//		 gheara_toggle=true;
			//	 }
			//	 okg=true;
			// }
			// ---
			
			// --- Moving lift
			// uncomment:
			// if (gamepad2.dpad_up) {
			// 	lift.moveLift(Constants.LiftTargets.HIGH);
			// } else if (gamepad2.dpad_right) {
			// 	lift.moveLift(Constants.LiftTargets.LOW);
			// } else if (gamepad2.dpad_left) {
			// 	lift.moveLift(Constants.LiftTargets.MEDIUM);
			// } else if (gamepad2.dpad_down) {
			// 	// belt.moveBelt(Constants.IntakeTargets.UP);
			// 	lift.moveLift(Constants.LiftTargets.PICKUP);
			// }
			// else if (gamepad2.right_bumper) {
			// 	lift.moveLift(400);
			// }
			// else if (gamepad2.right_stick_y>0.3){
			// 	lift.moveLift(200);
			// }
			// else if (gamepad2.right_stick_y<-0.3){
			// 	lift.moveLift(100);
			// }
			// ---
 
			// --- servouri gheara
			
			// uncomment:
			// if (gamepad2.left_bumper){
			// 	am_con=!am_con;
			// 	TimeUnit.MILLISECONDS.sleep(150);
			// 	// if(!am_con) { // reset, ca sa nu mai dai pe abxy
			// 	//	 servo2.setPower(0.0);
			// 	//	 servo3.setPower(0.0);
			// 	// }
			// }
			
			// if(am_con){
			// 	// servo2.setPower(0.1);
			// 	// servo3.setPower(-0.1);
			// 	// TimeUnit.MILLISECONDS.sleep(150);
			// 	servo2.setPower(0.04);
			// 	servo3.setPower(0.04);
			// }
			// else {
			// if (gamepad2.x)
			// 	servo2.setPower(0.2);
			// else if (gamepad2.a)
			// 	servo2.setPower(-0.2);
			// else servo2.setPower(0.0);
			
			// if (gamepad2.y)
			// 	servo3.setPower(-0.2);
			// else if (gamepad2.b)
			// 	servo3.setPower(0.2);
			// else servo3.setPower(0.0);
			// }
			
			
			
			// ---
 
 
			// --- gamepad 1 directie
			
			// motorFrontLeft.setPower(frontLeftPower);
			// motorBackLeft.setPower(backLeftPower);
			// motorFrontRight.setPower(frontRightPower);
			// motorBackRight.setPower(backRightPower);
			
			double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
			double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
			double rightX = -gamepad1.right_stick_x;
			final double v1 = r * Math.cos(robotAngle) + rightX;
			final double v2 = r * Math.sin(robotAngle) - rightX;
			final double v3 = r * Math.sin(robotAngle) + rightX;
			final double v4 = r * Math.cos(robotAngle) - rightX;

			motorFrontLeft.setPower(v1);
			motorFrontRight.setPower(v2);
			motorBackLeft.setPower(v3);
			motorBackRight.setPower(v4);
			
			
			// ---
			
			// uncomment:
			// telemetry.addData("Mode", "running");
			// telemetry.addData("servo1 power", " = " + servo1.getPower());
			// telemetry.addData("servo2 power", " = " + servo2.getPower());
			// telemetry.addData("servo3 power", " = " + servo3.getPower());
			// telemetry.addData("Lift Position", lift.getPosition());
			// telemetry.addData("A2", gamepad2.a); // gheara
			// // -- lift
			// telemetry.addData("Dpad2 Up", gamepad2.dpad_up);
			// telemetry.addData("Dpad2 right", gamepad2.dpad_right);
			// telemetry.addData("Dpad2 down", gamepad2.dpad_down);
			// telemetry.addData("Dpad2 left", gamepad2.dpad_left);
			// // --
			// telemetry.update();
		}
	}
}

