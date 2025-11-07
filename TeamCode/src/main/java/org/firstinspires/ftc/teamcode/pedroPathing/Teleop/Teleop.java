package org.firstinspires.ftc.teamcode.pedroPathing.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.RobotHardware.Turret;

@TeleOp(name = "Teleoperado")
public class Teleop extends OpMode {

    Turret turret;

    @Override
    public void init() {
        turret = new Turret(hardwareMap, 1);
    }

    @Override
    public void loop() {

    }
}
