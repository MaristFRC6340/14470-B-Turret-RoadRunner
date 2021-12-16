/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Modified by michaudc 2017
Additional modifications by michaudc 2021
*/
package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
//deez
/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="MaristBot2021: Teleop Quad 2021", group="Training")
//@Disabled
public class TeleopTank_Quad_2021 extends OpMode {

    /* Declare OpMode members. */
    Marist_BaseRobotQuad_2021 robot   = new Marist_BaseRobotQuad_2021(); // use the class created to define a Robot's hardware
    // could also use HardwarePushbotMatrix class.
    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo

    double robotSpeed = .7;
    double turnSpeed = .3;

    boolean armOn = false;

    int armPos = 0; //Vertical Angle of Arm
    int armAng = 0; //Horizontal Angle of Arm

    private void relaxArm() {
        robot.armLift.setPower(0);
        robot.armSpin.setPower(0);
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Robot Ready");    //

        // Set to Run without Encoder for Tele Operated
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double leftX;
        double leftY;
        double rightX;

        // Strafer Mode
        leftX = gamepad1.left_stick_x;
        leftY = gamepad1.left_stick_y;
        rightX = gamepad1.right_stick_x*turnSpeed;

        double leftRearPower = leftY + leftX - rightX;
        double leftFrontPower = leftY - leftX - rightX;
        double rightRearPower = leftY - leftX + rightX;
        double rightFrontPower = leftY + leftX + rightX;

        robot.leftFront.setPower(robotSpeed*leftFrontPower);
        robot.leftRear.setPower(robotSpeed*leftRearPower);
        robot.rightFront.setPower(robotSpeed*rightFrontPower);
        robot.rightRear.setPower(robotSpeed*rightRearPower);
        double left;
        double right;

        // Alternative Method:  Single Paddle on right  (Commented out)
        //left = gamepad1.right_stick_y + gamepad1.right_stick_x;
        //right = gamepad1.right_stick_y - gamepad1.right_stick_x;
        //robot.leftMotor.setPower(left);
        //robot.rightMotor.setPower(right);

        // Use gamepad left & right Bumpers to open and close the claw
//        if (gamepad1.right_bumper)
//            clawOffset += CLAW_SPEED;
//        else if (gamepad1.left_bumper)
//            clawOffset -= CLAW_SPEED;

        // Move both servos to new position.  Assume servos are mirror image of each other.
//        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
//        robot.leftHand.setPosition(robot.MID_SERVO + clawOffset);
//        robot.rightHand.setPosition(robot.MID_SERVO - clawOffset);

        /** MOVES ARM UP AND DOWN */
        int armSpeed = 3; //Changes how fast arm moves up and down

        if (gamepad1.a) {
            armOn = true;
            armPos += armSpeed;
        }
        if (gamepad1.y) {
            armOn = true;
            armPos -= armSpeed;
        }

        /** MOVES ARM SIDE TO SIDE */
        double armSpinSpeed = 3; //Controls how fast arm moves side to side

        if(gamepad1.dpad_right) {
            armOn = true;
            armAng -= armSpinSpeed;
        }
        if(gamepad1.dpad_left) {
            armOn = true;
            armAng += armSpinSpeed;
        }

        //Moves arm to vertical position
        robot.armLift.setTargetPosition(armPos);
        robot.armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Moves arm to horizontal position
        robot.armSpin.setTargetPosition(armAng);
        robot.armSpin.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /** RELAXES ARM */
        if(gamepad1.right_bumper)
            armOn = false;

        /** ACTIVATES SPINNER */
        if(gamepad1.b) {
            robot.spinner.setPower(-.8);
        }
        else if(gamepad1.x)
            robot.spinner.setPower(.8);
        else {
            robot.spinner.setPower(0);
        }




        if(armOn) {
            robot.armLift.setPower(0.8);
            robot.armSpin.setPower(0.8);
        }
        else {
            relaxArm();
        }



        // Control Spinner with Right and Left Triggers
        /** ACTIVATES INTAKE */
        double intakeMotorPower = gamepad1.left_trigger - gamepad1.right_trigger;
        // Limit Power to -0.4 to 0.4
        if (intakeMotorPower > 0.8) {
            intakeMotorPower = 0.8;
        }

        if (intakeMotorPower < -0.4) {
            intakeMotorPower = -0.4;
        }

        robot.intake.setPower(intakeMotorPower);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
