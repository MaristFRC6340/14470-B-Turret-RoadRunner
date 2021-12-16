package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import android.graphics.Color;

/**
 * Created by michaudc on 10/8/2017.
 * Updated by michaudc on 29 July 2021
 * Additional code by Allesio Toniolo July 2021
 * Based on HardwarePushbot Code from the FTCRobotController resources
 *
 * This class models the physical structure of the robot with instances
 * of motors, servos, and sensors.
 *
 * The following are name assignments to be configured
 * on the RC Phone in the App.
 *
 * Motor channel: leftFront:        "leftfront"
 * Motor channel: rightFront:       "rightfront"
 * Motor channel: leftRear:          "leftrear"
 * Motor channel: rightRear:         "rightrear"
 * Motor channel: leftArm:          "leftarm"
 * Motor channel: rightArm:         "rightarm"
 * Servo Channel: leftHand:         "lefthand"
 * Servo Channel: rightHand:        "righthand"
 * Touch Sensor:  touch             "touch" ** Digital 1 in Config
 * Color Sensor:  colorSensor       "colorSensor"
 *
 */

public class Marist_BaseRobotQuad_2021 {
    /* Public Motors and Servos */
    //Drive Motors
    public DcMotor leftFront   = null;
    public DcMotor rightFront  = null;
    public DcMotor leftRear    = null;
    public DcMotor rightRear   = null;

    //Arm Motors
    public DcMotor armSpin    = null;
    public DcMotor armLift     = null;
    public DcMotor intake    = null;

    //Spinner
    public DcMotor spinner = null;

    public Servo leftHand   = null;

    /* Public Sensors */
    public DigitalChannel touch = null;
    public NormalizedColorSensor colorSensor = null;

    // Constants for Arm and Servo Operation
    public static final double MID_SERVO        =  0.5;
    public static final double ARM_UP_POWER     =  0.45;
    public static final double ARM_DOWN_POWER   = -0.45;

    // For Encoder Functions
    private double     COUNTS_PER_MOTOR_REV          = 1440 ;    // eg: TETRIX Motor Encoder
    private final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private double     WHEEL_DIAMETER_INCHES         = 4.0 ;     // For figuring circumference
    private double     COUNTS_PER_INCH               = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private double COUNTS_PER_DEGREE                 = COUNTS_PER_MOTOR_REV / 360;
    private double     DRIVE_SPEED                   = 1;
    private double     TURN_SPEED                    = .6;

    // Local OpMode members
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    // Constructor - leave this blank for now
    public Marist_BaseRobotQuad_2021 () {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors.  Assign Names that match the setup on the RC Phone
        /** Define and Initialize Motors - NAMES MUST MATCH SETUP ON PHONE */
        //Drive Motors
        leftFront   = hwMap.dcMotor.get("leftfront");
        rightFront  = hwMap.dcMotor.get("rightfront");
        leftRear     = hwMap.dcMotor.get("leftrear");
        rightRear    = hwMap.dcMotor.get("rightrear");
        //Arm Motors
        armLift      = hwMap.dcMotor.get("armlift");
        armSpin     = hwMap.dcMotor.get("armspin");
        intake      = hwMap.dcMotor.get("intake");
        //Other Motors
        spinner     = hwMap.dcMotor.get("spinner");

        /** Set Direction of Motors */
        //Drive
        leftFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightFront.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftRear.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightRear.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        //Arm
        armSpin.setDirection(DcMotor.Direction.FORWARD);
        armLift.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        //Other
        spinner.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        //Drive
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        //Arm
        armSpin.setPower(0);
        armLift.setPower(0);
        intake.setPower(0);
        //Other
        spinner.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        //Drive
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Arm
        armSpin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //***** NO SERVOS INSTALLED YET *****\\

        // Define and initialize ALL installed servos.
//        leftHand = hwMap.servo.get("lefthand");
//        rightHand = hwMap.servo.get("righthand");
//        leftHand.setPosition(MID_SERVO);
//        rightHand.setPosition(MID_SERVO);

        //***** NO SENSORS INSTALLED YET *****\\

        // Define and Initialize Sensors
//        touch = hwMap.get(DigitalChannel.class, "touch");
//        colorSensor = hwMap.get(NormalizedColorSensor.class, "colorSensor");
//
//        touch.setMode(DigitalChannel.Mode.INPUT);

    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    // Additional Functions to control Servos and motors

    public void driveStraightInches(double speed,
                                    double inches,
                                    double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // Reverse inches
        inches = inches * -1;

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set to Limit of DRIVE_SPEED
        if (Math.abs(speed) > DRIVE_SPEED) {
            speed = DRIVE_SPEED; //
        }

        // Ensure that the opmode is still active
        //if (opModeIsActive()) {
        if (true) {       // Swapped out to include in MaristBaseRobot

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newLeftRearTarget = leftRear.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightRearTarget = rightRear.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

            //
            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            leftRear.setTargetPosition(newLeftRearTarget);
            rightRear.setTargetPosition(newRightRearTarget);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            period.reset();
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftRear.setPower(Math.abs(speed));
            rightRear.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((period.seconds() < timeoutS) &&
                    (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy() )) {
                // Wait for Sequence to complete
            }

            // Stop all motion;
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void pointTurnDegrees(double speed,
                                 double deg,
                                 double timeoutS) {

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // Reverse inches
        deg = deg * -2;

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set to Limit of DRIVE_SPEED
        if (Math.abs(speed) > DRIVE_SPEED) {
            speed = DRIVE_SPEED; //
        }

        // Ensure that the opmode is still active
        //if (opModeIsActive()) {
        if (true) {       // Swapped out to include in MaristBaseRobot

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(deg * COUNTS_PER_DEGREE);
            newRightFrontTarget = rightFront.getCurrentPosition() - (int)(deg * COUNTS_PER_DEGREE);
            newLeftRearTarget = leftRear.getCurrentPosition() + (int)(deg * COUNTS_PER_DEGREE);
            newRightRearTarget = rightRear.getCurrentPosition() - (int)(deg * COUNTS_PER_DEGREE);

            //
            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            leftRear.setTargetPosition(newLeftRearTarget);
            rightRear.setTargetPosition(newRightRearTarget);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            period.reset();
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftRear.setPower(Math.abs(speed));
            rightRear.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((period.seconds() < timeoutS) &&
                    (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy() )) {
                // Wait for Sequence to complete
            }

            // Stop all motion;
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

    }

    public void armMotorDeg(double speed,
                            double deg,
                            double timeoutS) {
        int target;

        armSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armSpin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set to Limit of DRIVE_SPEED
        if (Math.abs(speed) > DRIVE_SPEED) {
            speed = DRIVE_SPEED; //
        }

        // Ensure that the opmode is still active
        //if (opModeIsActive()) {
        if (true) {       // Swapped out to include in MaristBaseRobot

            // Determine new target position, and pass to motor controller
            target = armSpin.getCurrentPosition() + (int)(deg * COUNTS_PER_DEGREE);
            armSpin.setTargetPosition(target);

            // Turn On RUN_TO_POSITION
            armSpin.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            period.reset();
            armSpin.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((period.seconds() < timeoutS) &&
                    armSpin.isBusy()) {
                // Wait for Sequence to complete
            }

            // Stop all motion;
            armSpin.setPower(0);

            // Turn off RUN_TO_POSITION
            armSpin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

    }

    public void armLiftMotorDeg(double speed,
                                double deg,
                                double timeoutS) {
        int target;

        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set to Limit of DRIVE_SPEED
        if (Math.abs(speed) > DRIVE_SPEED) {
            speed = DRIVE_SPEED; //
        }

        // Ensure that the opmode is still active
        //if (opModeIsActive()) {
        if (true) {       // Swapped out to include in MaristBaseRobot

            // Determine new target position, and pass to motor controller
            target = armLift.getCurrentPosition() + (int)(deg * COUNTS_PER_DEGREE);
            armLift.setTargetPosition(target);

            // Turn On RUN_TO_POSITION
            armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            period.reset();
            armLift.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((period.seconds() < timeoutS) &&
                    armLift.isBusy()) {
                // Wait for Sequence to complete
            }

            // Stop all motion;
            armLift.setPower(0);

            // Turn off RUN_TO_POSITION
            armLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

    }

    // For 2020: Add Commands from API for Engineering Java Robot Simulation
    public void moveDistance(double inches, double speed) {
        driveStraightInches(speed, inches, 10);
    }

    public void turnAngle(double angle, double speed) {
        pointTurnDegrees(speed, angle, 10);
    }

    public void rotateArm(double angle, double speed) {
        armMotorDeg(speed, angle, 10);
    }

    public void liftArm(double angle, double speed) {
        armLiftMotorDeg(speed, angle, 10);
    }


    // Functions for Color Sensor - July 2021

    public float [] getColorValues() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float [] output = new float [3];
        output[0] = colors.red;
        output[1] = colors.blue;
        output[2] = colors.green;
        return output;
    }

    public double getRed() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        return colors.red;
    }

    public double getGreen() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        return colors.green;
    }

    public double getBlue() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        return colors.blue;
    }

    public float getIntensity() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);
        return hsvValues[1];
    }


}

