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
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.REVChassisHardwareSetup;

@Autonomous(name="REVChassis_AutonomousWarehouseRed", group="Concept")
//@Disabled
public class REVChassisAutoWarehouseRed extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    /* Define Hardware setup */
    // assumes left motors are reversed
    REVChassisHardwareSetup robot     =   new REVChassisHardwareSetup();
    /**
     * Constructor
     */
    public REVChassisAutoWarehouseRed() {
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);  //Initialize hardware from the HardwareHolonomic Setup

        //adds feedback telemetry to DS
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        /************************
         * Autonomous Code Below://
         *************************/
        //RaiseArm(100, 1000);
        DriveForwardTime(-DRIVE_POWER, 350);
        SpinLeft(DRIVE_POWER, 800);
        DriveForwardTime(-DRIVE_POWER * 2, 1000);
        StopDrivingTime(1000);
      /*  SpinLeft(DRIVE_POWER, 175);
        StopDrivingTime(1000);

        DriveForwardTime(-DRIVE_POWER, 275);
        DriveForwardTime(DRIVE_POWER, 40);
        StopDrivingTime(1000);

        OpenHand(750);
        StopDrivingTime(1000);*/




/* SpinRight(DRIVE_POWER, 250);
        StopDrivingTime(500);
        DriveForwardTime(-DRIVE_POWER, 1000);
        DriveForwardTime(DRIVE_POWER, 40);*/
        //|Extra Code for later if need be...|//
        /*DriveForwardTime(DRIVE_POWER, 1000); //neg power drives backwards
        DriveForwardTime(-DRIVE_POWER, 40);
        StopDrivingTime(1000);*/

       /* StrafeLeft(DRIVE_POWER, 1000);
        StrafeRight(DRIVE_POWER, 40);
        SpinLeft(DRIVE_POWER, 1000);
        StopDrivingTime(1000);*/
        /*StrafeRight(DRIVE_POWER, 1150);
        StrafeLeft(DRIVE_POWER, 40);
        StopDrivingTime(100);*/

/* currently no Servo configured on bot
        //RaiseArm();


        StopDrivingTime(1000);
        SpinRight(DRIVE_POWER/2, 4000);
        StopDrivingTime(1000);


        SpinLeft(DRIVE_POWER/2, 4000);
        StopDrivingTime(1000);

        StopDriving();
*/
    }//runOpMode

    /** Below: Basic Drive Methods used in Autonomous code...**/
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
        // write the values to the motors
        robot.motorFR.setPower(power);
        robot.motorFL.setPower(-power);
        robot.motorRR.setPower(-power);
        robot.motorRL.setPower(power);
        Thread.sleep(time);
    }

    public void StrafeRight(double power, long time) throws InterruptedException
    {
        StrafeLeft(-power, time);
    }

    public void SpinRight (double power, long time) throws InterruptedException
    {
        // write the values to the motors
        robot.motorFR.setPower(-power);
        robot.motorFL.setPower(power);
        robot.motorRR.setPower(-power);
        robot.motorRL.setPower(power);
        Thread.sleep(time);
    }

    public void SpinLeft (double power, long time) throws InterruptedException
    {
        SpinRight(-power, time);
    }


 //Currently no Servo configured in Holonomic Hardware setup
/*
    public void RaiseArm( double power, int pos) throws InterruptedException
    {
        robot.motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorArm.setTargetPosition(pos);
        robot.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorArm.setPower(power);
        while(robot.motorArm.isBusy())
        {
            telemetry.addData("armPos" , robot.motorArm.getCurrentPosition());
            telemetry.update();
        }
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
    }*/


}//TestAutoDriveByTime
