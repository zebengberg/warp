package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

@TeleOp
public class EncoderTester extends LinearOpMode {
    DcMotor motor;

    @Override
    public void runOpMode() {
        motor = hardwareMap.dcMotor.get("motor");
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while(opModeIsActive()){
            if (gamepad1.dpad_up) {
                motor.setPower(1);
            } else if (gamepad1.dpad_down) {
                motor.setPower(-1);
            } else {
                motor.setPower(0);
            }

            telemetry.addData("position", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
