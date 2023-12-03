package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Drivetrain.TankDrivetrain;


@TeleOp(name = "TankDrive")
public class TankDrive extends LinearOpMode {
    public TankDrivetrain robot = new TankDrivetrain();

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize hardware map
        robot.init(hardwareMap);
//        robot.larm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.l_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.r_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double leftPower;
        double rightPower;


        waitForStart();

        while (opModeIsActive()) {

            double drive = -gamepad1.left_stick_y * 0.7;
            double turn = -gamepad1.right_stick_x * 0.7;
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);

            if (drive > 0) {
                robot.l_motor.setPower(leftPower);
                robot.r_motor.setPower(rightPower);
            } else {
                robot.l_motor.setPower(rightPower);
                robot.r_motor.setPower(leftPower);
            }

            if (gamepad2.dpad_down) { // arm intake position
                robot.larm.setTargetPosition(307);
                //servo wrist intake position
                robot.larm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.larm.setPower(Math.abs(.3));
            } else if (gamepad2.dpad_left) { // arm default position
                robot.larm.setTargetPosition(430);
                //servo default position
                robot.larm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.larm.setPower(Math.abs(.3));
            } else if (gamepad2.dpad_up) {  // set pixel default position
                robot.larm.setTargetPosition(520);
                //servo wrist set pixel position
                robot.larm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.larm.setPower(Math.abs(.3));
            }

            if (gamepad2.right_bumper) { //lift set pixel position
                //robot.lift.setTargetPosition(-80);
                //robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //robot.lift.setPower(Math.abs(0.5));
            }
            if (gamepad2.left_bumper) { //lift get pixel position
                //robot.lift.setTargetPosition(-80);
                //robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //robot.lift.setPower(Math.abs(0.5));
            }

            if (gamepad1.right_stick_button && gamepad1.left_stick_button) {
                //plane launcher
                robot.plauncher.setPosition(1); // plane launcher position
            }
            if (gamepad2.a) { //get a pixels
                robot.left_pixel.setPosition(.1);

            }
            if (gamepad2.b) { //release a pixels
                robot.right_pixel.setPosition(1);
            }
            if (gamepad1.x) {
                robot.openleftClaw();
            }
            if (gamepad1.y) {
                robot.closeleftClaw();
            }

            if(gamepad2.left_bumper){
                robot.left_pixel.scaleRange(0, 1);
            }


            telemetry.addData("Left Pow", robot.l_motor.getPower());
            telemetry.addData("Left Encoder", robot.l_motor.getCurrentPosition());
            telemetry.addData("Right Pow", robot.r_motor.getPower());
            telemetry.addData("Right Encoder", robot.r_motor.getCurrentPosition());
            telemetry.addData("Core hex position", robot.larm.getCurrentPosition());
            telemetry.addData("Claw Servo position", robot.left_pixel.getPosition());
            telemetry.addData("Wrist Servo position", robot.wrist.getPosition());
            telemetry.addData("Plane Servo position", robot.plauncher.getPosition());
            telemetry.update();
        }

    }
}


//    public void encoderDrive(double speed,
//                             double leftInches, double rightInches,
//                             double timeoutS) {
//        int newLeftTarget;
//        int newRightTarget;
//
//        // Ensure that the OpMode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            newLeftTarget = robot.l_motor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
//            newRightTarget = robot.r_motor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
//            robot.l_motor.setTargetPosition(newLeftTarget);
//            robot.r_motor.setTargetPosition(newRightTarget);
//
//            // Turn On RUN_TO_POSITION
//            robot.l_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.r_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            robot.l_motor.setPower(Math.abs(speed));
//            robot.r_motor.setPower(Math.abs(speed));
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while (opModeIsActive() &&
//                    (runtime.seconds() < timeoutS) &&
//                    (leftDrive.isBusy() && rightDrive.isBusy())) {
//
//                // Display it for the driver.
//                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
//                telemetry.addData("Currently at",  " at %7d :%7d",
//                        leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
//                telemetry.update();
//            }
//        }
//    }


