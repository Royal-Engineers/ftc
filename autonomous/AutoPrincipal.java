/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
// import com.qualcomm.robotcore.hardware.ColorSensor;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;
// import com.qualcomm.robotcore.hardware.DistanceSensor;

// import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// import java.util.Locale;

import org.firstinspires.ftc.teamcode.subsystems.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Autonomous(name = 
	"🟦🟦🟦🟦🟦🟦🟦🟦🟦⬛⬛⬛⬛⬛🟦🟦🟦🟦🟦🟦🟦\n" +
	"🟦🟦🟦🟦🟦🟦🟦🟦⬛⬛🟥🟥🟥⬛⬛⬛🟦🟦🟦🟦🟦\n" +
	"🟦🟦🟦🟦🟦🟦🟦⬛⬛🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦🟦\n" +
	"🟦🟦🟦🟦🟦🟦🟦⬛🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦🟦\n" +
	"🟦🟦🟦🟦🟦🟦🟦⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
	"🟦🟦🟦🟦🟦🟦🟦⬛🟥🟥🟥⬜⬜⬜⬜⬜⬜⬛⬛🟦🟦\n" +
	"🟦🟦🟦🟦⬛⬛⬛🟥🟥🟥⬜⬜⬜⬜⬜⬜⬜⬜⬛🟦🟦\n" +
	"🟦🟦🟦⬛🟥⬛⬛🟥🟥🟥⬜⬜⬜⬜⬜⬜⬜⬜⬛🟦🟦\n" +
	"🟦🟦🟦⬛🟥⬛⬛🟥🟥🟥⬛⬜⬜⬜⬜⬜⬜⬜⬛🟦🟦\n" +
	"🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥⬛⬛⬛⬛⬛⬛⬛🟦🟦🟦\n" +
	"🟦🟦🟦🟦🟦⬛⬛⬛🟥🟥⬛⬛⬛🟥🟥🟥⬛🟦🟦🟦🟦\n" +
	"🟦🟦🟦🟦🟦🟦🟦⬛⬛⬛⬛🟦⬛⬛⬛⬛⬛🟦🟦🟦🟦", group = "Robot")
public class AutoPrincipal extends LinearOpMode {
	static final double COUNTS_PER_MOTOR_REV = 537.69; // nice
	static final double WHEEL_DIAMETER_INCHES = 3.78; // 96 mm diametru
	static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * 3.1415);
	static final double CSPEED = 0.4; // viteza "constanta" pentru motoare

	private DcMotor motorFrontLeft = null;
	private DcMotor motorFrontRight = null;
	private DcMotor motorBackLeft = null;
	private DcMotor motorBackRight = null;
	
	// --- senzor de culoare
	// ColorSensor sensorColor;
	// DistanceSensor sensorDistance;
	// ---

	// --- init motoare: hardwaremap; reverse FR, BR; zeropower; encoder reset; run
	// using encoder
	// DcMotor motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
	// DcMotor motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
	// DcMotor motorFrontRight= hardwareMap.get(DcMotor.class, "motorFrontRight");
	// DcMotor motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");

	@Override
	public void runOpMode() {
		Lift lift = new Lift();
		
		motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
		motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
		motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
		motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

		motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
		motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

		// -- astea nu ar trebui dar mai bine sa fie
		motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		// --

		motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		
		Servo servo_gheara = hardwareMap.servo.get("servo_gheara");

		// ---

		// Wait for the game to start (driver presses PLAY)
		// https://github.com/REVrobotics/Color-Sensor-v3-Examples/blob/master/Java/Color%20Match/src/main/java/frc/robot/Robot.java

		// sensorColor = hardwareMap.get(ColorSensor.class, "senzor_culoare");

		// // get a reference to the distance sensor that shares the same name.
		// sensorDistance = hardwareMap.get(DistanceSensor.class, "senzor_dist");

		// // hsvValues is an array that will hold the hue, saturation, and value
		// information.
		// float hsvValues[] = {0F, 0F, 0F};

		// // values is a reference to the hsvValues array.
		// final float values[] = hsvValues;

		// // sometimes it helps to multiply the raw RGB values with a scale factor
		// // to amplify/attentuate the measured values.
		// final double SCALE_FACTOR = 255;

		// // get a reference to the RelativeLayout so we can change the background
		// // color of the Robot Controller app to match the hue detected by the RGB
		// sensor.
		// int relativeLayoutId =
		// hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id",
		// hardwareMap.appContext.getPackageName());
		// final View relativeLayout = ((Activity)
		// hardwareMap.appContext).findViewById(relativeLayoutId);

		waitForStart();

		lift.init(hardwareMap);

		// JOYSTICK
			// fata = -
			// spate = +
			// stanga = -
			// dreapta = +
		
		// --- start sequence: se misca pana la conul cu culori
		servo_gheara.setPosition(Range.clip(0.08, 0, 1)); // inchis
		// servo_gheara.setPosition(Range.clip(0.135, MIN_POSITION, MAX_POSITION)); // deschis
		sleep(500);
		lift.moveLift(1000);
		//encoderDrive(0, -1, 0, 37.5, 0.2);
		// encoderDrive(1, 0, 0, 10);  // dreapta
		// encoderDrive(0, 1, 0, 10);  // spate
		// encoderDrive(-1, 0, 0, 10); // stanga
		// encoderDrive(0, -1, 0, 10); // fata
		// lift.moveLift(Constants.LiftTargets.PICKUP);
		encoderDrive(0, -1, 0, 5, 0.3); //merge fata
		encoderDrive(1 ,  0, 0, 6.5, 0.3); //merge dreapta
		
		
		encoderDrive(0, -1, 0, 29, 0.6); // merge fata
		encoderDrive(0, -1, 0, 3, 0.35);
		ColorSensor color_sensor;
		color_sensor = hardwareMap.colorSensor.get("sensor_color");
		color_sensor.enableLed(true);  // Turn the LED on
	
		telemetry.addData("red", color_sensor.red());   // Red channel value
		telemetry.addData("green", color_sensor.green()); // Green channel value
		telemetry.addData("blue", color_sensor.blue());  // Blue channel value
 
		telemetry.addData("alpha", color_sensor.alpha()); // Total luminosity
		telemetry.addData("argb", color_sensor.argb());  // Combined color value
		// telemetry.update();
		 // dupa asta citeste iar

		sleep(100);

		telemetry.addData("red", color_sensor.red());   // Red channel value
		telemetry.addData("green", color_sensor.green()); // Green channel value
		telemetry.addData("blue", color_sensor.blue());  // Blue channel value
 
		telemetry.addData("alpha", color_sensor.alpha()); // Total luminosity
		telemetry.addData("argb", color_sensor.argb());  // Combined color value
		// telemetry.update();
		int zona = 0;
		int v_red   = color_sensor.red();
		int v_green = color_sensor.green();
		int v_blue  = color_sensor.blue();
		if (v_green>v_red && v_green>v_blue)
			zona = 2;
		else if (v_red>v_blue && v_red>v_green)
			zona = 3;
		else if (v_blue>v_green && v_blue>v_red)
			zona = 1;
		telemetry.addData("zona", zona);
		telemetry.update();
		sleep(200);
		color_sensor.enableLed(false); // Turn the LED off
		
		
		
		
		
		encoderDrive(0, -1, 0, 60.3, 0.55); // merge fata
		
		// ---
		
		// ---
		// switch(zona)
		// {
		// 	case 1: 
		// 		encoderDrive(-1, 0, 0, 37.5, 0.2);
			
		// 	case 2:
		// 		break;
				
		// 	case 3:
		// 		encoderDrive(1, 0, 0, 37.5, 0.2);
				
		// 	default:
		// 		break;
				
		// }
		sleep(50);
		encoderDrive(0, 1, 0, 20, 0.55); // merge in spate
		encoderDrive(0, 0, 1, 10, 0.555); // intoarce spre dreapta

		lift.moveLift(3800); // ridica lift
		sleep(1600);
		encoderDrive(0, -1, 0, 10, 0.3); // merge in fata
		sleep(200);
		servo_gheara.setPosition(Range.clip(0.13, 0, 1)); // deschis
		sleep(50);
		encoderDrive(0, 1, 0, 13, 0.3); // spate
		//sleep(250);
		//servo_gheara.setPosition(Range.clip(0.08, 0, 1)); // inchis
		sleep(100);
		lift.moveLift(750); // coboara lift
		sleep(50);
		
		encoderDrive(0, 0, -1, 28.2, 0.554); //invarte stanga
		encoderDrive(0,  -1, 0, 31.5, 0.54); // fata
		sleep(50);
		servo_gheara.setPosition(Range.clip(0.08, 0, 1)); // inchis
		
		
		// 2.4cm 1 unitatea imagibnara
		// encoderDrive(-1,  0, 0, 10); // fata
		// encoderDrive(-1 ,  1, 0, 10); // dreapta
		// encoderDrive(1 ,  1, 0, 10); // spate
		// encoderDrive(1 , -1, 0, 10); // stanga
		
		// convert the RGB values to HSV values.
		// multiply by the SCALE_FACTOR.
		// then cast it back to int (SCALE_FACTOR is a double)
		// Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
		// (int) (sensorColor.green() * SCALE_FACTOR),
		// (int) (sensorColor.blue() * SCALE_FACTOR),
		// hsvValues);

		// // send the info back to driver station using telemetry function.
		// telemetry.addData("Distance (cm)",
		// String.format(Locale.US, "%.02f",
		// sensorDistance.getDistance(DistanceUnit.CM)));
		// telemetry.addData("Alpha", sensorColor.alpha());
		// telemetry.addData("Red ", sensorColor.red());
		// telemetry.addData("Green", sensorColor.green());
		// telemetry.addData("Blue ", sensorColor.blue());
		// telemetry.addData("Hue", hsvValues[0]);

		// // change the background color to match the color detected by the RGB sensor.
		// // pass a reference to the hue, saturation, and value array as an argument
		// // to the HSVToColor method.
		// relativeLayout.post(new Runnable() {
		// public void run() {
		// relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
		// }
		// });

		// telemetry.update();
		// }

		// // Set the panel back to the default color
		// relativeLayout.post(new Runnable() {
		// public void run() {
		// relativeLayout.setBackgroundColor(Color.WHITE);
		// }
		// });

		//sleep(1000); // pause to display final telemetry message.
	}

	public void encoderDrive(double leftv, double rightv, double rotatie, double lungime, double viteza) {
		// lungime = in inch; de inmultit cu COUNTS_PER_INCH

		if (opModeIsActive()) {
			// daca nu e activ => nasol
			
			// JOYSTICK
			// fata = -
			// spate = +
			// stanga = -
			// dreapta = +
			
			double r = Math.hypot(-leftv, rightv);
			double robotAngle = Math.atan2(rightv, -leftv) - Math.PI / 4;
			double rightX = -rotatie;
			final double v1 = r * Math.cos(robotAngle) + rightX;
			final double v2 = r * Math.sin(robotAngle) - rightX;
			final double v3 = r * Math.sin(robotAngle) + rightX;
			final double v4 = r * Math.cos(robotAngle) - rightX;
			// vn = modifier

			// nu stiu daca ma lasa sa pun long aici, poate trebuie double, dar in fine...
			// vom vedea
			int v1f = (int) (v1 * lungime * COUNTS_PER_INCH) + motorFrontLeft.getCurrentPosition();
			int v2f = (int) (v2 * lungime * COUNTS_PER_INCH) + motorFrontRight.getCurrentPosition();
			int v3f = (int) (v3 * lungime * COUNTS_PER_INCH) + motorBackLeft.getCurrentPosition();
			int v4f = (int) (v4 * lungime * COUNTS_PER_INCH) + motorBackRight.getCurrentPosition();

			motorFrontLeft.setTargetPosition(v1f);
			motorFrontRight.setTargetPosition(v2f);
			motorBackLeft.setTargetPosition(v3f);
			motorBackRight.setTargetPosition(v4f);

			motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

			// runtime.reset();

			motorFrontLeft.setPower(viteza);
			motorFrontRight.setPower(viteza);
			motorBackLeft.setPower(viteza);
			motorBackRight.setPower(viteza);

			while (opModeIsActive()
					/* && (runtime.seconds() < timeoutS) */
					&& (motorFrontLeft.isBusy() || motorFrontRight.isBusy() ||
							motorBackLeft.isBusy() || motorBackRight.isBusy())) {
				// telemetrie in bucla
				// telemetry.addData("Merg la front:", " %7d :%7d", v1f, v2f);
				// telemetry.addData("Merg la back:", "%7d :%7d", v3f, v4f);
				// telemetry.addData("Acum sunt front:", " %7d :%7d", motorFrontLeft.getCurrentPosition(),
				// 		motorFrontRight.getCurrentPosition());
				// telemetry.addData("Acum sunt back:", " %7d :%7d", motorBackLeft.getCurrentPosition(),
				// 		motorBackRight.getCurrentPosition());
				// telemetry.update();
			}

			// // Stop all motion;
			motorFrontLeft.setPower(0);
			motorFrontRight.setPower(0);
			motorBackLeft.setPower(0);
			motorBackRight.setPower(0);

			// Turn off RUN_TO_POSITION
			motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		}
	// optional pause after each move.
	} // end functie encoderdrive
}
