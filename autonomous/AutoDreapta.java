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
import org.firstinspires.ftc.robotcore.external.Const;
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
public class AutoDreapta extends LinearOpMode {
	static final double COUNTS_PER_MOTOR_REV = 537.69; // nice
	static final double WHEEL_DIAMETER_INCHES = 3.78; // 96 mm diametru
	static final double CONSTANTA_SABIN = 1.9; // nu intreba
	static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * 3.1415)) / CONSTANTA_SABIN;
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

		waitForStart();

		lift.init(hardwareMap);

		// JOYSTICK
			// fata = -
			// spate = +
			// stanga = -
			// dreapta = +
		
		servo_gheara.setPosition(Range.clip(0.08, 0, 1));
		sleep(250);
		lift.moveLift(100);
		sleep(200);
		
		encoderDrive( 0, -1, 0, 5, 0.5, 500); // fata
		encoderDrive(-1,  0, 0, 17, 0.5, 1000); // stanga
		encoderDrive( 0, -1, 0, 21, 0.5, 1500); // fata, ajuns la robot
		
		ColorSensor color_sensor;
		color_sensor = hardwareMap.colorSensor.get("sensor_color");
		color_sensor.enableLed(true);  // Turn the LED on
	
		telemetry.addData("red", color_sensor.red());   // Red channel value
		telemetry.addData("green", color_sensor.green()); // Green channel value
		telemetry.addData("blue", color_sensor.blue());  // Blue channel value
 
		telemetry.addData("alpha", color_sensor.alpha()); // Total luminosity
		telemetry.addData("argb", color_sensor.argb());  // Combined color value

		sleep(500);

		telemetry.addData("red", color_sensor.red());   // Red channel value
		telemetry.addData("green", color_sensor.green()); // Green channel value
		telemetry.addData("blue", color_sensor.blue());  // Blue channel value
 
		telemetry.addData("alpha", color_sensor.alpha()); // Total luminosity
		telemetry.addData("argb", color_sensor.argb());  // Combined color value
		
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
		
		sleep(300);
		
		color_sensor.enableLed(false); // Turn the LED off

		encoderDrive( 0, -1, 0, 31+45, 0.5, 4000); // fata; ~40cm e robotul
		encoderDrive(-1,  0, 0, 30, 0.5, 1500); // stanga, am HIGH junction in fata
		lift.moveLift(3900); // lift sus, HIGH+100
		sleep(5000);
		encoderDrive( 0, -1, 0, 21, 0.2, 1000); // fata, punem conul dupa
		servo_gheara.setPosition(Range.clip(0.13, 0, 1));
		sleep(200);
		servo_gheara.setPosition(Range.clip(0.08, 0, 1));
		encoderDrive( 0,  1, 0, 21, 0.2, 1000); // inapoi
		lift.moveLift(50); // trebuie dat la ~0 la sfarsitul de auto, pentru manual
		encoderDrive( 1,  0, 0, 30, 0.5, 1500); // dreapta, inapoi in mijloc la C5
		encoderDrive( 0,  1, 0, 60, 0.5, 3000); // -1 tile, merge la B5
		
		switch(zona)
		{
			case 1: {
				encoderDrive(-1, 0, 0, 60, 0.5, 3000);
				break;
			}
			case 2: {
				break;
			}			
			case 3: {
				encoderDrive(1, 0, 0, 60, 0.5, 3000);
				break;
			}
				
			default: // nasol
				break;
		}
	
	}

	public void encoderDrive(double leftv, double rightv, double rotatie, double lungime, double viteza, long timp) {
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

			// while (opModeIsActive()
					/* && (runtime.seconds() < timeoutS) */
					// && (motorFrontLeft.isBusy() || motorFrontRight.isBusy() ||
					// 		motorBackLeft.isBusy() || motorBackRight.isBusy())
							// ) {
								// ;
				// telemetrie in bucla
				// telemetry.addData("Merg la front:", " %7d :%7d", v1f, v2f);
				// telemetry.addData("Merg la back:", "%7d :%7d", v3f, v4f);
				// telemetry.addData("Acum sunt front:", " %7d :%7d", motorFrontLeft.getCurrentPosition(),
				// 		motorFrontRight.getCurrentPosition());
				// telemetry.addData("Acum sunt back:", " %7d :%7d", motorBackLeft.getCurrentPosition(),
				// 		motorBackRight.getCurrentPosition());
				// telemetry.update();
			// }

			sleep(timp);

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
