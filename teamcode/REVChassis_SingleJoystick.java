/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/*
 * This file contains a configuration for Mr. Reynolds' TestBed motors/servos/sensor
 *
 * It is intended to test basic function of program parameters
 *
 * Note: this Class uses a TestHardwareTeleOp configuration file.
 *
 * You could make a copy and adjust Configuration to match your bot for use as a basic testing
 * platform.
 */


@TeleOp(name="REVChassis_SingleJoystick", group="Examples")  // @Autonomous(...) is the other common choice
@Disabled
public class REVChassis_SingleJoystick extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    REVChassisHardwareSetup robot = new REVChassisHardwareSetup();  // Use MyBotHardware Setup

    int     armHoldPosition;             // reading of arm position when buttons released to hold

    double axisY;
    double axisX;
    double leftVal;
    double rightVal;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);                //Initialize hardware from the MyBotHardware Setup
        //Now open Wobble Servos after init


        //display telemetry before hitting play
        while (!opModeIsActive()) { // Testing out loop to continuted update of armPos while waiting for start
            //init current position of arm motor
            //armHoldPosition = robot.motorArm.getCurrentPosition();
            //adds feedback telemetry to DS
            telemetry.addData("Status", "OpMode Not Started");
           // telemetry.addData("armPosition: ", +robot.motorArm.getCurrentPosition());
            telemetry.addData("Hold: ", + armHoldPosition);
            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
        }
      /*  runtime.reset();
        *//*
         * TeleOp Code Below:
         * NOTE: all code must go inside this while loop
         *
         *//*
        while (opModeIsActive())   // run until the end of the match (driver presses STOP)
        {
            //adds feedback telemetry to DS
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Status", "OpMode Active");
            telemetry.addData("armPosition: ", +robot.motorArm.getCurrentPosition());
            telemetry.addData("Hold: ", + armHoldPosition);
            telemetry.addData("gamePadY: ", +gamepad1.left_stick_y);


            // JoyStick control
            axisY = gamepad1.left_stick_y;
            axisX = gamepad1.right_stick_x;

            //may need to change + and - to make robot turn Left and Right correctly

            leftVal = axisY - axisX;
            rightVal = axisX + axisY;
            //Code for when one or both sticks slows down the driving speed of the bot
            if (gamepad1.right_stick_button && gamepad1.left_stick_button) {
                robot.motorFL.setPower(leftVal / 4);
                robot.motorFR.setPower(rightVal / 4);
                robot.motorRR.setPower(rightVal / 4);
                robot.motorRL.setPower(leftVal / 4);
            } else if (gamepad1.right_stick_button) {
                robot.motorFL.setPower(leftVal / 2);
                robot.motorFR.setPower(rightVal / 2);
                robot.motorRR.setPower(rightVal / 2);
                robot.motorRL.setPower(leftVal / 2);
            } else {
                robot.motorFL.setPower(leftVal);
                robot.motorFR.setPower(rightVal);
                robot.motorRR.setPower(rightVal);
                robot.motorRL.setPower(leftVal);
            }

            if (gamepad1.right_bumper) {
                robot.motorArm.setPower(-gamepad1.right_trigger/2);
            }
            else {
                robot.motorArm.setPower(gamepad1.right_trigger/2);
            }
            if (gamepad1.a) {
                robot.servoHandRight.setPosition(robot.OPEN);
            } else if (gamepad1.b) {
                robot.servoHandRight.setPosition(robot.CLOSED);
            }*/
    idle();
    }
}
