package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.external.samples.Config;

class leftfront {} class rightFront{
    public static void setDirection(DcMotorSimple.Direction reverse) {
    }
}

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Config
public class TeleOp extends OpMode {
    @Override
    public void init() {
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "left_front");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "right_front");

        // Set motor directions and zero power behaviors
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     *
     */
    @Override
    public void loop() {

    }

}