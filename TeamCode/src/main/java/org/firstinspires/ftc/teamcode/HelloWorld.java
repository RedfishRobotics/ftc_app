package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name="Library Test", group="Linear Opmode")

public class HelloWorld extends LinearOpMode {

    Auto_Library autoLibrary = new Auto_Library();
    TeleOp_Library teleOpLibrary = new TeleOp_Library();

    @Override
    public void runOpMode() {
        teleOpLibrary.driveAndStrafe();
        //This is a test of Git
        //Another test
    }
}
