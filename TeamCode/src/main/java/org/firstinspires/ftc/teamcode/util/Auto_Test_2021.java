package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name="Auto_Test_Camera", group="Training")
//@Disabled
public class Auto_Test_2021 extends LinearOpMode {

    /* Declare OpMode members. */
    Marist_BaseRobotQuad_2021 robot   = new Marist_BaseRobotQuad_2021();
    private ElapsedTime runtime = new ElapsedTime();

    // Variables to control Speed
    double velocity = 0.5; // Default velocity



    //Camera
    WebcamName webcamName;
    OpenCVCamera camera;

    //Pipeline
    Pipeline_Target_Detect myPipeLine = new Pipeline_Target_Detect();

    //Zones
    int zone = 1;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        /** TEST WEBCAM */
        webcamName = hardwareMap.get(WebcameName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);

        //Initialize Pipeline
        camera.setPipeline(myPipeLine);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        /**Asynchronusly Open Camera */
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            public void onOpened() {
                //Start streaming from camera
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                //Initialize Pipeline
                camera.setPipeline(myPipeline);
            }
            public void onError(int errorCode) {

            }
        });



        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        double xPos = -1;

        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < 4)) {
            xPos = myPipeline.getXPos();
            telemetry.addData("XPos", xPos);
            telemetry.update();
        }

        if(xPos > 175) {
            zone = 3;
        } else if (xPos > 75) {
            zone = 2;
        }

        telemetry.addData("XPos", xPos);
        telemetry.addData("Zone", zone);
        telemetry.update();

        //shows xpos and zone
        delay(5);

        //Stop the camera
        camera.stopStreaming();
        camrea.closeCameraDevice();

        // Perform the steps of Autonomous One Step at a Time.
        // MARIST: Add Code here!
        // Available Calls: forward(inches),delay(seconds),right(degrees),left(degrees)
        // robot.leftHand.setPosition(), robot.rightHand.setPosition()
        // Engineering Java Calls:
        // robot.moveDistance(inches, speed)
        // robot.turnAngle(degrees, speed)





        // Autonomous Finished
        telemetry.addData("Path", "Complete");
        telemetry.update();
        //sleep(1000);
    }

    // Functions for REACH 2019 based on Python Turtles
    public void forward(double inches)
    {
        robot.driveStraightInches(velocity, inches, 10);
    }

    public void right(double degrees)
    {
        robot.pointTurnDegrees(velocity, degrees, 10);
    }

    public void left(double degrees)
    {
        degrees = degrees * -1;
        robot.pointTurnDegrees(velocity, degrees, 10);
    }

    public void speed(int speed)
    {
        double newSpeed = (double)speed / 10.0;
        velocity = newSpeed;
    }

    // Sample Delay Code
    public void delay(double t) { // Imitates the Arduino delay function
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    // REACH: Add Functions Here


}
