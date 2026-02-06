package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "odo test")
public class EncoderTestOp extends OpMode {

    DcMotorEx odo;
    @Override
    public void init() {
        odo = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        odo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        telemetry.addData("test", odo.getCurrentPosition());
    }
}
