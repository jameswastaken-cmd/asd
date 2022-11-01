/*
   Holonomic/Mecanum concept autonomous program. Driving motors for TIME

   Robot wheel mapping:
          X FRONT X
        X           X
      X  FL       FR  X
              X
             XXX
              X
      X  BL       BR  X
        X           X
          X       X
*//*

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="REVChassisIMUTest", group="Concept")
@Disabled
public class REVChassisAutoIMUTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    */
/* Define Hardware setup *//*

    // assumes left motors are reversed
    REVChassisHardwareSetup robot     =   new REVChassisHardwareSetup();

    */
/**
     * Constructor
     *//*

    public REVChassisAutoIMUTest() {
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);  //Initialize hardware from the HardwareHolonomic Setup

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        */
/************************
         * Autonomous Code Below://
         *************************//*






*/
/* SpinRight(DRIVE_POWER, 250);
        StopDrivingTime(500);
        DriveForwardTime(-DRIVE_POWER, 1000);
        DriveForwardTime(DRIVE_POWER, 40);*//*

        //|Extra Code for later if need be...|//
        */
/*DriveForwardTime(DRIVE_POWER, 1000); //neg power drives backwards
        DriveForwardTime(-DRIVE_POWER, 40);
        StopDrivingTime(1000);*//*


       */
/* StrafeLeft(DRIVE_POWER, 1000);
        StrafeRight(DRIVE_POWER, 40);
        SpinLeft(DRIVE_POWER, 1000);
        StopDrivingTime(1000);*//*

        */
/*StrafeRight(DRIVE_POWER, 1150);
        StrafeLeft(DRIVE_POWER, 40);
        StopDrivingTime(100);*//*


*/
/* currently no Servo configured on bot
        //RaiseArm();


        StopDrivingTime(1000);
        SpinRight(DRIVE_POWER/2, 4000);
        StopDrivingTime(1000);


        SpinLeft(DRIVE_POWER/2, 4000);
        StopDrivingTime(1000);

        StopDriving();
*//*

    }//runOpMode

    */
/** Below: Basic Drive ethods used in Autonomous code...**//*

    //set Drive Power variable
    double DRIVE_POWER = 0.5;

    public void DriveForward(double power)
    {
        // write the values to the motors
        robot.motorFR.setPower(power);//still need to test motor directions for desired movement
        robot.motorFL.setPower(power);
        robot.motorRR.setPower(power);
        robot.motorRL.setPower(power);
    }

    public void DriveForwardTime(double power, long time) throws InterruptedException
    {
        DriveForward(power);
        Thread.sleep(time);
    }

    public void StopDriving()
    {
        DriveForward(0);
    }

    public void StopDrivingTime(long time) throws InterruptedException
    {
        DriveForwardTime(0, time);
    }

    public void StrafeLeft(double power, long time) throws InterruptedException
    {
        //while degree is less than desired
            robot.motorFR.setPower(power);
            robot.motorFL.setPower(-power);
            robot.motorRR.setPower(-power);
            robot.motorRL.setPower(power);

        //stop motors
    }



 //Currently no Servo configured in Holonomic Hardware setup

    public void RaiseArm( double power, long time) throws InterruptedException
    {
        robot.motorArm.setPower(power); //note: uses servo instead of motor.
        Thread.sleep(time);
        robot.motorArm.setPower(0.0);
    }



    public void StopDrivingArm() {DriveForward(0);}
    public void StopDrivingArmTime(long time)
    {
        robot.motorArm.setPower(0.0);
        sleep(time);
    }
    public void LowerArm( double power, long time) throws InterruptedException
    {
        robot.motorArm.setPower(power);
        sleep(time);
        robot.motorArm.setPower(0.0);
    }
    public void CloseHand(long time)
    {
        robot.servoHandLeft.setPosition(0.9);
        robot.servoHandRight.setPosition(-0.9);
        sleep(time);
    }

    public void OpenHand(long time)
    {
        robot.servoHandLeft.setPosition(-0.9);
        robot.servoHandRight.setPosition(0.9);
        sleep(time);
    }


}//TestAutoDriveByTime
*/
