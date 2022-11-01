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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;

import java.util.List;

@Autonomous(name="REVChassis_AutoCone", group="Concept")
//@Disabled
public class REVautonomouscone1 extends LinearOpMode
{
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


    private static final String[] LABELS =
            {
                    "placeholder 1",
                    "placeholder 2",
                    "placeholder 3"
            };
    private static final String VUFORIA_KEY =
            "ATm94PP/////AAABmWqS4Gad+ki+jNEC8eEg4gBZXzDCzg5OqOfOOld8G2jvFFc8QEMLFNVjWKnepq3vkgFzaifwq/AGdZ6wjGX6J+1RMLGRAZZXpkOOThg+WHlzhSUfxc9+TBC3+RNLHOfXUMVy+BDuiJ0+vAzKt7vdXDcHVn6jTJ5/jtXg1gjqCgKdQnxUBWSHURvofqpWJ+xoa1fQj0ELZWsfmWNe/a/OmPxyaLi5wgW+2A6sIk6ewW0umMKxLaT4h/YjzMPVqEj6EZEwZyfWm5a7+xM8vTc9sw6WkQdZfBZ3ObogT1oMDC6nIj17OlppESMqWThLEyCuehbshLrw9uesiSRoAgtd5WN5AwIroIVEF54Tj1cffeXi";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public void initVuforia()
    {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    public void initTfod()
    {
        int tfodMontitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMontitorViewId);
        tfodParameters.minResultConfidence = 0.78f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters,vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        tfod.setZoom(2, 1.56);
    }


    private ElapsedTime runtime = new ElapsedTime();
    /* Define Hardware setup */
    // assumes left motors are reversed
    REVChassisHardwareSetup robot     =   new REVChassisHardwareSetup();
    /**
     * Constructor
     */

    public String checkConeImage()
    {
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null)
            {
                for (Recognition recognition : updatedRecognitions)
                {
                    if (recognition.getConfidence() > .78)
                    {
                        if (recognition.getLabel().equals("placeholder 1"))
                        {
                            return "placeholder 1";
                        }
                    }
                }
            }
        }
        return "placeholder 1";
    }


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);  //Initialize hardware from the HardwareHolonomic Setup();
        initVuforia();
        initTfod();
        //adds feedback telemetry to DS
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        if(tfod != null)
        {
            tfod.activate();
        }



        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        String coneImage = checkConeImage();

        telemetry.addData("Image: ", coneImage);
        telemetry.update();
        /************************
         * Autonomous Code Below://
         *************************/

        if(coneImage.equals("placeholder 1"))
        {
            DriveForwardTime(.5, 500);
        }
        else if(coneImage.equals("placeholder 2"))
        {
            DriveForwardTime(.5 , 500);
        }
        else if(coneImage.equals("placeholder 3"))
        {
            DriveForwardTime(.5, 500);
        }
        /*double armMinPos = 0 ;      // encoder position for arm at bottom
        double armMaxPos = 3100.0;   // encoder position for arm at top
        double armLowPos = 100;     //encoder position for arm at low
        double medianPos = 500;   //encoder position for arm in middle
        int armHoldPosition;             // reading of arm position when buttons released to hold
        double slopeVal = 1000.0;   // increase or decrease to perfect holding power

        telemetry.addData("Status", "Autonomous in motion.");

        telemetry.update();

        int target = (int) armMaxPos;
        robot.motorArm.setTargetPosition(target);
        robot.motorArm.setPower(.5);
        while(robot.motorArm.getCurrentPosition() < target)
        {
            robot.motorArm.setPower(.5);
        }
        armHoldPosition = target;*/

        //SpinRight(DRIVE_POWER, 800);
        //DriveForwardTime(-DRIVE_POWER, 750);
        //StrafeLeft(DRIVE_POWER, 500);
        //DriveForwardTime(-DRIVE_POWER * 2, 1000);
        //StopDrivingTime(1000);
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
        robot.motorFR.setPower(-power);//still need to test motor directions for desired movement
        robot.motorFL.setPower(power);
        robot.motorRR.setPower(-power);
        robot.motorRL.setPower(power);
    }

    public void DriveForwardTime(double power, long time) throws InterruptedException
    {
        DriveForward(-power);
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
