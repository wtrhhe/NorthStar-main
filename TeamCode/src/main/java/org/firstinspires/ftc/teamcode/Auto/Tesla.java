package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Drivetrain.TankDrivetrain;
import org.firstinspires.ftc.teamcode.Vision.TeamElementSubsystem;

@Autonomous(name="Tesla", group="Auto")

public class Tesla extends LinearOpMode{

    public TankDrivetrain robot = new TankDrivetrain();
    public int element_zone = 1;

    private TeamElementSubsystem teamElementDetection=null;

    boolean togglePreview = true;

    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();

        teamElementDetection = new TeamElementSubsystem(hardwareMap);

        telemetry.addData("Object Creation", "Done");
        telemetry.update();
    }



    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        double leftPower;
        double rightPower;

        HardwareStart();

        String curAlliance = "red";
        String curSide = "Backdrop side";

        while (!opModeIsActive() && !isStopRequested()){
            element_zone = teamElementDetection.elementDetection(telemetry);
            if (togglePreview && gamepad2.a){
                togglePreview = false;
                teamElementDetection.toggleAverageZone();
            }else if (!gamepad2.a){
                togglePreview = true;
            }


            if (gamepad1.x){
                curAlliance = "blue";
            }else if (gamepad1.b) {
                curAlliance = "red";
            }

            if (gamepad2.x){
                curSide = "BackDrop Side";
            }else if (gamepad2.b){
                curSide = "PlaneField Side";
            }
            teamElementDetection.setAlliance(curAlliance);
            teamElementDetection.setSide(curSide);
            telemetry.addData("-------------------------------------------------------------", "");
            telemetry.addData("Select Alliance :", "");
            telemetry.addData("Gamepad1 X = Blue, Gamepad1 B = Red", "");
            telemetry.addData("Select Side :", "");
            telemetry.addData("Gamepad2 X = BackDrop Side","");
            telemetry.addData("Gamepad2 B = PlaneField Side","");
            telemetry.addData("-------------------------------------------------------------", "");
            telemetry.addData("Current Alliance Selected : ", curAlliance.toUpperCase());
            telemetry.addData("Current Side Selected : ", curSide.toUpperCase());
            telemetry.addData("Claw Position: ", robot.left_pixel.getPosition());

            telemetry.update();
        }

        if(opModeIsActive()){
            if(curAlliance == "red" && curSide == "PlaneField Side") {
                if (element_zone == 2) {
                    sleep(500);
                    moveRobot(2900, .3);

                    sleep(500);
                    robot.openleftClaw();

                    sleep(500);
                    turnRobot("left", 90);

                    sleep(500);
                    moveRobot(24000, -.3);
                    // Claw open/close and wrist/arm up code in there
                }
                if (element_zone == 1){
                    sleep(500);
                    turnRobot("left", 30);

                    sleep(500);
                    moveRobot(2900, .3);

                    // Claw open/close code in there

                    sleep(500);
                    turnRobot("left", 90);

                    sleep(500);
                    moveRobot(24000, -.3);
                    // Claw open/close and wrist/arm up code in there
                }
                if(element_zone == 3){
                    sleep(500);
                    turnRobot("right", 30);

                    sleep(500);
                    moveRobot(2900, .3);

                    // Claw open/close code in there

                    sleep(500);
                    turnRobot("left", 90);

                    sleep(500);
                    moveRobot(24000, -.3);
                    // Claw open/close and wrist/arm up code in there
                }
            }

            if(curAlliance == "red" && curSide == "BackDrop Side"){
                if(element_zone == 1){
                    sleep(500);
                    turnRobot("left", 15);

                    sleep(500);
                    moveRobot(2900, .3);

                    // Claw open/close code in there

                    sleep(500);
                    turnRobot("left", 25);

                    sleep(500);
                    moveRobot(14000, -.3);
                    // Claw open/close and wrist/arm up code in there
                }
                if(element_zone == 2){
                    sleep(500);
                    moveRobot(2900, .3);

                    // Claw open/close code in there

                    sleep(500);
                    turnRobot("left", 30);

                    sleep(500);
                    moveRobot(10000, -.3);
                    // Claw open/close and wrist/arm up code in there
                }
                if(element_zone == 3){
                    sleep(500);
                    turnRobot("right", 15);

                    sleep(500);
                    moveRobot(2900, .3);

                    // Claw open/close code in there

                    sleep(500);
                    turnRobot("left", 40);

                    sleep(500);
                    moveRobot(8000, -.3);

                    // Claw open/close and wrist/arm up code in there
                }
            }


        }

        telemetry.addData("Object", "Passed waitForStart");

        telemetry.update();

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