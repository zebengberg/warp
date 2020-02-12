package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Random;


// TeleOp for minibot.

@Autonomous
public class MiniBotRoomba extends LinearOpMode {

    private DcMotor left_motor;
    private DcMotor right_motor;
    private DcMotor[] motors = {left_motor, right_motor};

    private DistanceSensor left_distance;
    private DistanceSensor right_distance;

    @Override
    public void runOpMode() {

        left_motor = hardwareMap.dcMotor.get("left_motor");
        right_motor = hardwareMap.dcMotor.get("right_motor");
        for (DcMotor motor : motors) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        left_distance = hardwareMap.get(DistanceSensor.class, "left_distance");
        right_distance = hardwareMap.get(DistanceSensor.class, "right_distance");

        waitForStart();
        while (opModeIsActive()) { explore(); }
    }

    private void printStatus() {
        telemetry.addData("left motor", left_motor.getPower());
        telemetry.addData("right motor", right_motor.getPower());
        telemetry.addData("left distance", left_distance.getDistance(DistanceUnit.CM));
        telemetry.addData("right distance", right_distance.getDistance(DistanceUnit.CM));
        telemetry.update();
    }

    private int getRandom(int min, int max) {
        Random r = new Random();
        return r.nextInt(max - min) + min;
    }

    private double getDistance() {
        return Math.min(left_distance.getDistance(DistanceUnit.CM), right_distance.getDistance(DistanceUnit.CM));
    }

    private void randomCW() {
        int start = left_motor.getCurrentPosition();
        int rand = getRandom(100, 500);
        while (left_motor.getCurrentPosition() < start + rand) {
            left_motor.setPower(1);
            right_motor.setPower(-1);
            printStatus();
        }
    }

    private void randomCCW() {
        int start = right_motor.getCurrentPosition();
        int rand = getRandom(100, 500);
        while (right_motor.getCurrentPosition() < start + rand) {
            right_motor.setPower(1);
            left_motor.setPower(-1);
            printStatus();
        }
    }

    private void goForward(int x) {
        int start = right_motor.getCurrentPosition();
        while ((right_motor.getCurrentPosition() < start + x) && (getDistance() > 50)) {
            right_motor.setPower(1);
            left_motor.setPower(1);
            printStatus();
        }
        right_motor.setPower(0);
        left_motor.setPower(0);
    }

    private void goBack(int x) {
        int start = right_motor.getCurrentPosition();
        while (right_motor.getCurrentPosition() < start + x) {
            right_motor.setPower(-0.5);
            left_motor.setPower(-0.5);
            printStatus();
        }
        right_motor.setPower(0);
        left_motor.setPower(0);
    }

    private void explore() {
        int rand = getRandom(5000, 15000);
        goForward(rand);
        randomCCW();
    }
}


