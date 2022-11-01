
package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * Created by TeameurekaRobotics on 12/30/2016
 *
 * This file contains an example Hardware Setup Class.
 *
 * It can be customized to match the configuration of your Bot by adding/removing hardware, and then used to instantiate
 * your bot hardware configuration in all your OpModes. This will clean up OpMode code by putting all
 * the configuration here, needing only a single instantiation inside your OpModes and avoid having to change configuration
 * in all OpModes when hardware is changed on robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 *
 */

public class REVChassisHardwareSetup {

   /* Declare Public OpMode members.
    *these are the null statements to make sure nothing is stored in the variables.
    */

    //motors
    public DcMotor motorFR = null;
    public DcMotor motorFL = null;
    public DcMotor motorRR = null;
    public DcMotor motorRL = null;
    //public DcMotor slide = null;
    public DcMotor motorArm = null;
   /*
    public DcMotor motorWheel = null;

    //servos
    public Servo servoHandRight = null;
    public Servo servoHandLeft = null;
    public Servo servoHandUp = null;
    *//* public Servo Lifter = null;
    public Servo Hook = null;
    public Servo SPusher = null;
    public Servo Puller = null; *//*

    //sensors
   // public OpticalDistanceSensor lightSensor;   //  Modern Robotics ODS sensor
    //public TouchSensor rumbleTouch = null;
*/      public TouchSensor magTouch = null;

    //parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
    //variables
   /*( static final double     WHITE_THRESHOLD = 0.2;  // spans between 0.1 - 0.5 from dark to light
    static final double     APPROACH_SPEED  = 0.3;  // adjust to desired motor speed ( 0.0 - 1.0 ) */

    /* local OpMode members. */
    HardwareMap hwMap        = null;

    //Create and set default servo positions & MOTOR STOP variables.
    //Possible servo values: 0.0 - 1.0  For CRServo 0.5=stop greater or less than will spin in that direction
    //MOTOR VARIABLES
    final static double MOTOR_STOP = 0.0; // sets motor power to zero
    //final static double SLOPE = 500; // Stops the arm from dropping
    double  armMinPos        = 20;      // encoder position for arm at bottom
    double  armMaxPos        = 5000.0;   // encoder position for arm at top
    double armMedianPos = 500;
    double armLowPos = 100;
    double armFloor = 20;
    //double  armHoldPosition;             // reading of arm position when buttons released to hold

    //SERVO VARIABLES
    //Launch platform
    final static double UP = 0.925;
    final static double MID = 0.89;
    final static double DOWN = 0.15;
    final static double PEG = 0.875;
    //Wobble grabber
    /*final static double CLOSEED = .99;
    final static double OPEEN = 0.1;*/
    //Pusher
    final static double BACKWARD = 0.88;
    final static double FORWARD = 0.78;
    //Servo Puller
    final static double CLOSED = .1;
    final static double OPEN = .50;
    final static double  Go = 1.0;
    final static double  Stop = 0.5;

    //Servo Arm
    final static double Up = 0.5;
    final static double Down = 0.0;



    /* Constructor   // this is not required as JAVA does it for you, but useful if you want to add
    * function to this method when called in OpModes.
    */
    public REVChassisHardwareSetup() {
    }

    //Initialize standard Hardware interfaces
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        /************************************************************
         * MOTOR SECTION
         ************************************************************/
        // Define Motors to match Robot Configuration File
        motorFL = hwMap.dcMotor.get("motorFL");
        motorFR = hwMap.dcMotor.get("motorFR");
        motorRR = hwMap.dcMotor.get("motorRR");
        motorRL = hwMap.dcMotor.get("motorRL");
        //slide = hwMap.dcMotor.get("Slide");
        motorArm = hwMap.dcMotor.get("arm");
        //motorWheel = hwMap.dcMotor.get("motorWheel");

        // Set the drive motor directions:
        motorFR.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorFL.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        motorRR.setDirection(DcMotor.Direction.FORWARD); // Can change based on motor configuration
        motorRL.setDirection(DcMotor.Direction.FORWARD); // Can change based on motor configuration
        motorArm.setDirection(DcMotor.Direction.REVERSE);
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Keep the motors from moving during initialize.
        motorFL.setPower(MOTOR_STOP);
        motorFR.setPower(MOTOR_STOP);
        motorRR.setPower(MOTOR_STOP);
        motorRL.setPower(MOTOR_STOP);
        motorArm.setPower(MOTOR_STOP);


        // Set motors to run USING or WITHOUT encoders
        // Depending upon your configuration and use
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motorWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /************************************************************
         * SERVO SECTION
         ************************************************************/
        // shooter servos
        /*servoHandRight = hwMap.servo.get("servoHandRight");
        servoHandLeft = hwMap.servo.get("servoHandLeft");
        servoHandUp = hwMap.servo.get("servoHandUp");

        servoHandUp.setPosition(0.98);*/
        /*//wobble arm servos
        Hook = hwMap.servo.get("Hook");Puller = hwMap.servo.get("Puller");
        Hook.setPosition(CLOSED);

        Puller.setPosition(PULL);*/


        /************************************************************
         * SENSOR SECTION
         ************************************************************/
        //Define sensors

        magTouch = hwMap.get(TouchSensor.class, "maglimit");
        //rumbleTouch = hwMap.touchSensor.get("touch");


        // get a reference to ouBr Light Sensor object.
        //lightSensor = hwMap.opticalDistanceSensor.get("sensor_ods");  // Primary MR ODS sensor.

        // turn on LED of light sensor.
       // lightSensor.enableLed(true);
   }


}

