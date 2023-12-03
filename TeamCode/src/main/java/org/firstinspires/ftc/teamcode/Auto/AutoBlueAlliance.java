package org.firstinspires.ftc.teamcode.Auto;

import org.firstinspires.ftc.teamcode.Drivetrain.TankDrivetrain;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import java.awt.font.NumericShaper;
import java.util.Random;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import  com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Drivetrain.TankDrivetrain;
import org.firstinspires.ftc.teamcode.TeleOp.TankDrive;

@Autonomous(name="AutoBlueAlliance", group="Robot")
@Disabled
public class AutoBlueAlliance extends LinearOpMode{

    public TankDrivetrain robot = new TankDrivetrain();

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
        //forward
        robot.l_motor.setPower(-0.5);
        robot.r_motor.setPower(-0.5);
        sleep(700);

        //left
        robot.l_motor.setPower(0.5);
        robot.r_motor.setPower(-0.5);
        sleep(400);

        //left
//        robot.l_motor.setPower(-0.5);
//        robot.r_motor.setPower(0.5);
//        sleep(200);
    }

}