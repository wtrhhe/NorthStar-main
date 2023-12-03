/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Drivetrain.TankDrivetrain;
import org.firstinspires.ftc.teamcode.Vision.Pipeline.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "camera")
public class CameraWork extends LinearOpMode
{
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    public TankDrivetrain robot = new TankDrivetrain();
    enum Side{
        Right,
        Left
    }
    enum Alliance{
        Red,
        Blue
    }
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    //static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int Left = 1;
    int Middle = 2;
    int Right = 3;
    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {

        Side park = Side.Right;
        Alliance color = Alliance.Red;
        boolean isRight = true;
        boolean isRed = true;

        robot.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            // first press of a button
            previousGamepad1.copy(currentGamepad1);
            // second press of a button
            currentGamepad1.copy(gamepad1);

            robot.left_pixel.setPosition(1); //close the claw
            //robot.lClaw.setPosition(0);
            sleep(10);

            //write function to different positions of the arm
            robot.larm.setTargetPosition(500);
            //robot.rarm.setPosition(0.20);
            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper){
                isRed = !isRed;}

            if (isRed){color = Alliance.Red;}  else{ color = Alliance.Blue;}

            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
                isRight = !isRight;}

            if (isRight){park = Side.Right;}  else{ park = Side.Left;}

            telemetry.addData("Alliance",color);
            telemetry.addData("Side",park);

            telemetry.update();
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == Left || tag.id == Middle || tag.id == Right)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                        telemetry.addData("Alliance",color);
                        telemetry.addData("Side",park);
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                    telemetry.addData("Alliance",color);
                    telemetry.addData("Side",park);
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.addData("Alliance",color);
            telemetry.addData("Side",park);
            telemetry.update();
        }


        if(tagOfInterest == null || tagOfInterest.id == Left  ){
            //trajectory for LEFT
            telemetry.clear();
            sleep(500);          //wait half a second
            //robot.claw1.setPosition(1);     //close claw
            //robot.wrist.setPosition(0);     //fix wrist
            moveRobot(800,0.35);  // Forward 1 tile
            telemetry.addLine(" ID: 1 => Left | Moving forward");
            telemetry.update();


            sleep(500);
            turnRobot("left", 30);     //turn left for a degree
            telemetry.addLine(" ID: 1 => Left | Turning left");
            telemetry.update();
            //moveRobot(1500,0.35);  // Forward 1 tile


            //place purple hex
            sleep(1500);

            // lift claw

            //turn right
            turnRobot("right", 30);     //turn right for a degree
            telemetry.addLine(" ID: 1 => Left | Turning right");
            telemetry.update();

            //move forward
            moveRobot(1000,0.35);
            telemetry.addLine(" ID: 1 => Left | Moving forward");
            telemetry.update();

            //turn right
            turnRobot("right", 90);     //turn left for a degree
            telemetry.addLine(" ID: 1 => Left | Turning to the backdrop");
            telemetry.update();

            //move forward
            moveRobot(4000,0.35);     //turn left for a degree
            telemetry.addLine(" ID: 1 => Left | Turning left");
            telemetry.update();


            //sleep(6000);
//            turnRobot("left", 125);     //turn left for a degree
//            moveRobot(100,.75);
//            sleep(1000);
//
//
//            robot.claw1.setPosition(0);     //open claw
//            robot.wrist.setPosition(0);     //fix wrist
//            sleep(3000);         //wait 3 seconds
//            moveRobot(200,-1);  //moveRobot(x,y) forward for a x milliseconds at y speed
//            sleep(100);          //wait a 0.1 second
//
//            sleep(1000);                   //wait a 1 second
//            turnRobot("left",70);    //turn left for a degree
//            moveRobot(800,1);             //moveRobot(x,y) forward for a x milliseconds at y speed

        }else if(tagOfInterest.id == Middle){
            //trajectory for MIDDLE

        }else if(tagOfInterest.id == Right){
            //trajectory for RIGHT

        }

    }

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        ///while (opModeIsActive()) {sleep(20);}

    void tagToTelemetry(AprilTagDetection detection)
    {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
    }

    void moveRobot(double time, double speed) {
        robot.l_motor.setPower(-speed);
        robot.r_motor.setPower(-speed);
        sleep((long) (time));
        robot.l_motor.setPower(0);
        robot.r_motor.setPower(0);
    }

    void turnRobot(String direction, int degrees) {
        if (direction == "right") {
            robot.l_motor.setPower(-0.19);
            robot.r_motor.setPower(0.19);
            sleep((degrees/30)*500);
            robot.l_motor.setPower(0);
            robot.r_motor.setPower(0);
        }
        if (direction == "left") {
            robot.l_motor.setPower(0.2);
            robot.r_motor.setPower(-0.2);
            sleep((degrees/30)*500);
            robot.l_motor.setPower(0);
            robot.r_motor.setPower(0);
        }
    }
}

