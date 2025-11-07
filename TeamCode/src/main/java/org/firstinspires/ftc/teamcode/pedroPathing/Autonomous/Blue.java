package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.RobotHardware.RunAutonomous;
import org.firstinspires.ftc.teamcode.pedroPathing.RobotHardware.Turret;

@Autonomous(name = "Autonomous Blue")
public class Blue extends OpMode {

    RunAutonomous run;
    Turret turret;

    @Override
    public void init() {
        run = new RunAutonomous(true, hardwareMap);
        turret = new Turret(hardwareMap, 1);
    }

    @Override
    public void start() {
        run.startMode();
    }

    @Override
    public void loop() {
        run.autonomousPathUpdateBlue();


    }
}
