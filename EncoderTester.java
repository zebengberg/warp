package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class EncoderTester extends LinearOpMode {
    DcMotor motor;
    ColorSensor color;

    @Override
    public void runOpMode() {
        motor = hardwareMap.dcMotor.get("motor");
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        color = hardwareMap.get(ColorSensor.class, "color");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                motor.setPower(1);
            } else if (gamepad1.dpad_down) {
                motor.setPower(-1);
            } else {
                motor.setPower(0);
            }

            //telemetry.addData("position", motor.getCurrentPosition());
            //telemetry.update();
            printHSV();
        }
    }

    public void printHSV() {
        float[] hsv = {0F, 0F, 0F};
        Color.RGBToHSV(color.red() * 255, color.green() * 255, color.blue() * 255, hsv);
        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];
        telemetry.addData("red", color.red());
        telemetry.addData("gre", color.green());
        telemetry.addData("blu", color.blue());
        telemetry.addData("hue", hue);
        telemetry.addData("sat", sat);
        telemetry.addData("val", val);
        telemetry.update();
    }
}
