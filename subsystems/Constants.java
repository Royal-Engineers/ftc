/*
Copyright (c) 2014-2022 FIRST.  All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

package org.firstinspires.ftc.teamcode.subsystems;

// import com.acmerobotics.dashboard.config.Config;

// @Config("Constants")
public class Constants {
	/*
	Lift Constants
	*/
	public enum LiftTargets {
		PICKUP,
		LOW,
		MEDIUM,
		HIGH,
		PUTDOWN,
	} // PICKUP: 0, LOW: 100, MEDIUM: 300, HIGH: 550, PUTDOWN: 50
	// @TODO Verify if PUTDOWN should be swapped with PICKUP

	/*
	Belt constants
	*/
	public enum IntakeTargets {
		UP,
		HOLD,
		DOWN
	} // PICKUP: -261, HOLD: 0 , DROPOFF: -285

	/*
	Claw Constants
	*/
	public enum ClawTargets {
		OPENCLAW,
		CLOSECLAW
	} // OPENCLAW: 0, CLOSECLAW: 1

	/*
	Lift Init Variables
	*/
	public static int BELT_DOWN_POSITION = -250;
	public static int BELT_UP_POSITION = 250;// this is the difference! we go down 280, then we have to go back up 280 to get back to 0

	// @TODO: Verify claw limit with constants above
	public static double clawCloseLimit = .8;
	public static double clawOpenLimit = .4;

	public Constants() {}
}
