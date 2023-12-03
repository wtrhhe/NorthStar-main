package org.firstinspires.ftc.teamcode.Auto;

/**
 * This is an Auto Driving the Robot and during the presentation for explaining the possibilities of Robot
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.Drivetrain.TankDrivetrain;

@Autonomous(name="AutoRedAlliance", group="Robot")
@Disabled
public class AutoRedAlliance extends LinearOpMode {

    public TankDrivetrain robot = new TankDrivetrain();

    static final double WHITE_THRESHOLD = 0.5;
    static final double APPROACH_SPEED = 0.25;

    @Override
    public void runOpMode() {
//initialize hardware map
        robot.init(hardwareMap);

        double leftPower;
        double rightPower;

        waitForStart();

        while(opModeIsActive()) {
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
        }


    }

    private void moveRobot(double time, double speed) {
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
            sleep((degrees / 30) * 500);
            robot.l_motor.setPower(0);
            robot.r_motor.setPower(0);
        }
    }
}