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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains a configuration for Mr. Reynolds' TestBed motors/servos/sensor
 *
 * It is intended to test basic function of program parameters
 *
 * Note: this Class uses a TestHardwareTeleOp configuration file.
 *
 * You could make a copy and adjust Configuration to match your bot for use as a basic testing
 * platform.
 */



@TeleOp(name="REVDriveTrainArmTest", group="Competition")  // @Autonomous(...) is the other common choice
//@Disabled
public class REVDriveTrainArmTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    REVChassisHardwareSetup robot = new REVChassisHardwareSetup();  // Use MyBotHardware Setup
    // variables for arm limits and hold position
    // note: these can be placed in your hardwareSetup Class

    double armMinPos = 100 ;      // encoder position for arm at bottom
    double armMaxPos = 3450.0;   // encoder position for arm at top
    double armLowPos = 850;
    double medianPos = 2550;
    double armFloor = 20;//encoder position for arm in middle
    int armHoldPosition;             // reading of arm position when buttons released to hold
    double slopeVal = 1000.0;   // increase or decrease to perfect holding power


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);                //Initialize hardware from the MyBotHardware Setup

        // Wait for the game to start (driver presses PLAY)

        //init current position of arm motor
        // armHoldPosition = robot.motorArm.getCurrentPosition();


        //display telemetry before hitting play
        while (!opModeIsActive()) { // Testing out loop to continuted update of armPos while waiting for start

            //adds feedback telemetry to DS
            telemetry.addData("Status", "OpMode Not Started");
            //telemetry.addData("armPosition: ",  robot.motorArm.getCurrentPosition());
            telemetry.addData("Hold: ", armHoldPosition);
            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
        }

        runtime.reset();
        /************************
         * TeleOp Code Below://
         *************************/

        while (opModeIsActive()) {  // run until the end of the match (driver presses STOP)
            //drive region
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //if strafe doesn't work, switch around +/- on holonomic
            //(note: The joystick goes negative when pushed forwards)/
            // left stick: X controls Strafe & Y controls Spin Direction
            // right stick: Y controls drive Forward/Backward
            float gamepad1LeftY = -gamepad1.right_stick_x;   // drives spin left/right
            float gamepad1LeftX = gamepad1.left_stick_x;    // strafe direction (side to side)
            float gamepad1RightX = gamepad1.left_stick_y;  //drives forwards and backwards


            // holonomic formulas
            float FL = gamepad1RightX - gamepad1LeftX + gamepad1LeftY;
            float FR = -gamepad1RightX - gamepad1LeftX + gamepad1LeftY;
            float BR = -gamepad1RightX + gamepad1LeftX + gamepad1LeftY;
            float BL = gamepad1RightX + gamepad1LeftX + gamepad1LeftY;

            // clip the right/left values so that the values never exceed +/- 1
            FR = Range.clip(FR, -1, 1);
            FL = Range.clip(FL, -1, 1);
            BL = Range.clip(BL, -1, 1);
            BR = Range.clip(BR, -1, 1);

            // write the clipped values from the formula to the motors
            //super slow
            if (gamepad1.left_bumper) {
                robot.motorFR.setPower(FR / 4);
                robot.motorFL.setPower(FL / 4);
                robot.motorRL.setPower(BL / 4);
                robot.motorRR.setPower(BR / 4);
            }//fast
            else if (gamepad1.right_bumper) {
                robot.motorFR.setPower(FR);
                robot.motorFL.setPower(FL);
                robot.motorRL.setPower(BL);
                robot.motorRR.setPower(BR);
            }//normal speed
            else {
                robot.motorFR.setPower(FR / 2);
                robot.motorFL.setPower(FL / 2);
                robot.motorRL.setPower(BL / 2);
                robot.motorRR.setPower(BR / 2);
            }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //arm code region

            /**  old arm if using bumper
             * if(gamepad2.right_stick_y < 0.0 ) //&& robot.motorArm.getCurrentPosition() > armMinPos
             {
             robot.motorArm.setPower(gamepad2.right_stick_y/2); // let stick drive UP (note this is positive value on joystick)
             armHoldPosition = robot.motorArm.getCurrentPosition(); // while the lift is moving, continuously reset the arm holding position
             } else if (gamepad2.right_stick_y > 0.0 ) //&& robot.motorArm.getCurrentPosition() < armMaxPosencoder less than Max limit
             {
             robot.motorArm.setPower(gamepad2.right_stick_y/2); //let stick drive DOWN (note this is negative value on joystick)
             armHoldPosition = robot.motorArm.getCurrentPosition(); // while the lift is moving, continuously reset the arm holding position
             }  else //triggers are released - try to maintain the current position
             **/


            if(gamepad2.x)//encoder has to be at 0
            {
                int target = (int) armMinPos;//position to go to
                robot.motorArm.setTargetPosition(target);
                while(robot.motorArm.getCurrentPosition() >= target)
                {
                    robot.motorArm.setPower(1);
                    armHoldPosition = robot.motorArm.getCurrentPosition();
                }

               // robot.motorArm.setPower((double) (armHoldPosition - robot.motorArm.getCurrentPosition()) / slopeVal);
            }
            else if(gamepad2.b)//encoder has to be at 0
            {
                int target = (int) medianPos;//position to go to
                robot.motorArm.setTargetPosition(target);
                while(robot.motorArm.getCurrentPosition() < target)
                {
                    robot.motorArm.setPower(1);
                    armHoldPosition = robot.motorArm.getCurrentPosition();
                }

               // robot.motorArm.setPower((double) (armHoldPosition - robot.motorArm.getCurrentPosition()) / slopeVal);
            }
            else if(gamepad2.a)//encoder has to be at 0
            {
                int target = (int) armMaxPos;//position to go to
                robot.motorArm.setTargetPosition(target);
                while(robot.motorArm.getCurrentPosition() < target)
                {
                    robot.motorArm.setPower(1);
                    armHoldPosition = robot.motorArm.getCurrentPosition();
                }

                //.motorArm.setPower((double) (armHoldPosition - robot.motorArm.getCurrentPosition()) / slopeVal);
            }
            else if(gamepad2.y)//encoder has to be at 0
            {
                int target = (int) armMaxPos;//position to go to
                robot.motorArm.setTargetPosition(target);
                while(robot.motorArm.getCurrentPosition() < target)
                {
                    robot.motorArm.setPower(1);
                    armHoldPosition = robot.motorArm.getCurrentPosition();
                }

                //robot.motorArm.setPower((double) (armHoldPosition - robot.motorArm.getCurrentPosition()) / slopeVal);
            }
            else if(gamepad2.dpad_down)
            {
                int target = (int) armFloor;
                robot.motorArm.setTargetPosition(target);
                while(robot.motorArm.getCurrentPosition() < target)
                {
                    robot.motorArm.setPower(.4);
                    armHoldPosition = robot.motorArm.getCurrentPosition();
                }

                //robot.motorArm.setPower((double) (armHoldPosition - robot.motorArm.getCurrentPosition()) / slopeVal);
                /*while(!robot.magTouch.isPressed())
                {
                    robot.motorArm.setPower(-0.5);
                }
                robot.motorArm.setPower(0);
                armHoldPosition = robot.motorArm.getCurrentPosition();*/

            }
            /*se if(gamepad2.left_stick_y > 0.0 && gamepad2.left_stick_y < 0.0)
            {
                robot.motorArm.setPower(gamepad2.left_stick_y);
                armHoldPosition = robot.motorArm.getCurrentPosition();
            }
            else//hold position if none of the buttons as well as joystick
            {

                robot.motorArm.setPower((double) (armHoldPosition - robot.motorArm.getCurrentPosition()) / slopeVal);
            }*/ //*****************

            // Arm Control - Uses left joystick on gamepad2
            /*if (gamepad2.left_stick_y < 0.0) // encoder greater that lower limit && robot.motorArm.getCurrentPosition() < armMinPos
            {
                if (gamepad2.left_bumper)
                {
                    robot.motorArm.setPower(gamepad2.left_stick_y / 2);
                }
                robot.motorArm.setPower(-gamepad2.left_stick_y); // let stick drive UP (note this is positive value on joystick)
                armHoldPosition = robot.motorArm.getCurrentPosition(); // while the lift is moving, continuously reset the arm holding position
            } else if (gamepad2.left_stick_y > 0.0) //encoder less than Max limit && robot.motorArm.getCurrentPosition() > armMaxPos
            {
                if (gamepad2.left_bumper) {
                    robot.motorArm.setPower(gamepad2.left_stick_y / 2);
                }
                robot.motorArm.setPower(-gamepad2.left_stick_y); //let stick drive DOWN (note this is negative value on joystick)
                armHoldPosition = robot.motorArm.getCurrentPosition(); // while the lift is moving, continuously reset the arm holding position
            } else //triggers are released - try to maintain the current position

                robot.motorArm.setPower((double) (armHoldPosition - robot.motorArm.getCurrentPosition()) / slopeVal); //  // Note that if the lift is lower than desired position,
                // the subtraction will be positive and the motor will
                // attempt to raise the lift. If it is too high it will
                // be negative and thus try to lower the lift
                // adjust slopeVal to achieved perfect hold power
            }
*/

            // Hand controls = uses two continuous rotation servos for the hand
           /* if(gamepad2.a)
            {
                robot.servoHandRight.setPosition(0.9);
                robot.servoHandLeft.setPosition(0.1);
            }

            else if(gamepad2.y)
            {
                robot.servoHandRight.setPosition(0.1);
                robot.servoHandLeft.setPosition(0.9);
            }
            else
            {
                robot.servoHandRight.setPosition(0.5);
                robot.servoHandLeft.setPosition(0.5);
            }

            if(robot.rumbleTouch.isPressed())
            {
                gamepad1.rumble(0.9, 0.9, 200);
                gamepad2.rumble(0.9, 0.9, 200);
            }

            //caroseul spinner wheel
           if (gamepad2.x)
           {
               robot.motorWheel.setPower(0.75);
           }
           else if (gamepad2.b)
           {
               robot.motorWheel.setPower(-0.75);
           }
           else
               {
               robot.motorWheel.setPower(0.0);
               }

           //Tem8n\7 6trream Element extender servo
           if k(gamepad2.dpad_up)
           {/j/ extends
               robot.servoHandUp.setPosition(.98);
           }
           else if (gamepad2.dpad_left)
           {// second position
                robot.servoHandUp.setPosition(0.17);
                robot.servoHandUp.setPosition(0.20);
                robot.servoHandUp.setPosition(0.25);
           }
           else if (gamepad2.dpad_right)
           {// third position
               robot.servoHandUp.setPosition(0.10);
               robot.servoHandUp.setPosition(0.07);
           }
           else if (gamepad2.dpad_down)
           {// contracts
               robot.servoHandUp.setPosition(0.12);
           }

*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //new arm code, sensor code region
                if(!robot.magTouch.isPressed())//if magnet is not pressed
                {
                    if(gamepad2.left_stick_y > 0 || gamepad2.left_stick_y < 0)//joystick positive or negative
                    {
                        robot.motorArm.setPower(-gamepad2.left_stick_y); //drive motor
                        armHoldPosition = robot.motorArm.getCurrentPosition();//updates current hold position
                    }
                    else//hold motor
                    {

                        robot.motorArm.setPower((double) (armHoldPosition - robot.motorArm.getCurrentPosition()) / slopeVal);

                    }


                }
                if(robot.magTouch.isPressed())
                {

                    telemetry.addData("DETECTED", robot.magTouch);
                    robot.motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    armHoldPosition = robot.motorArm.getCurrentPosition();

                    if (gamepad2.left_stick_y < 0)//if joystick goes up, allow motor to drive
                    {
                        telemetry.addData("Joystick Y", gamepad2.left_stick_y);
                        robot.motorArm.setPower(-gamepad2.left_stick_y);
                        armHoldPosition = robot.motorArm.getCurrentPosition();

                    }


                }


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //telemetry region
            /*
             * Display Telemetry for debugging
             */
            //telemetry.addData("Text", "*** Robot Data***");
            //telemetry.addData("Joy1 XL YL XR", String.format("%.2f", gamepad1LeftX) + " " + String.format("%.2f", gamepad1LeftY) + " " + String.format("%.2f", gamepad1RightX));
            telemetry.addData("Joy2 YL",  String.format("%.2f", gamepad2.left_stick_y));
        /*    telemetry.addData("f left pwr", "front left  pwr: " + String.format("%.2f", FL));
            telemetry.addData("f right pwr", "front right pwr: " + String.format("%.2f", FR));
            telemetry.addData("b right pwr", "back right pwr: " + String.format("%.2f", BR));
            telemetry.addData("b left pwr", "back left pwr: " + String.format("%.2f", BL));*/
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("ArmPos: ",  + robot.motorArm.getCurrentPosition());
            telemetry.addData("ArmHold", +armHoldPosition);
            //telemetry.addData("servo" + robot.servoHandLeft());
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}