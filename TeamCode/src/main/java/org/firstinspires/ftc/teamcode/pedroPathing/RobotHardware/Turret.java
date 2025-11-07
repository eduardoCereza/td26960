package org.firstinspires.ftc.teamcode.pedroPathing.RobotHardware;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Configurable
public class Turret {
    //Move a Turret
    DcMotorEx corehex;

    //Flywheel
    DcMotorEx flywheel;

    //Intake
    DcMotorEx intake;

    //Transfer
    CRServo transfer;

    //Limelight
    Limelight3A limelight;

    //Variaveis do PID
    public static double kp = 0.075, kd = 0.000001, ki = 0.00001;
    double integralSum = 0, lastError = 0, derivate, error;
    ElapsedTime timer;

    //Inicializa componentes do robô
    public Turret(HardwareMap hwmap, int index){
        //Inicialização do motor que gira a Turret
        corehex = hwmap.get(DcMotorEx.class, "corehex");
        corehex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        corehex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Inicialização do motor da flywheel
        flywheel = hwmap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        //Inicialização motor do intake
        intake = hwmap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //Inicialização servo do transfer
        transfer = hwmap.get(CRServo.class, "transfer");
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        //Inicialização da limelight3A
        limelight = hwmap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(index);
        limelight.setPollRateHz(100);
        limelight.start();

        timer = new ElapsedTime();
    }

    //Alinha a Turret no centro ao goal
    public void align(boolean red) {
        int id = red ? 24 : 20;
        //Resultados da leitura da Limelight
        LLResult resultAlign = limelight.getLatestResult();

        //Focar nas tags que a gente quer
        List<LLResultTypes.FiducialResult> fiducialResults = resultAlign.getFiducialResults();

        //Loop
        if (resultAlign != null && resultAlign.isValid()) {

            //Tx
            double tx = resultAlign.getTx();

            //Posição atual do motor
            int currentPosition = corehex.getCurrentPosition();

            //PID
            error = -tx;

            derivate = (error - lastError) / timer.seconds();
            integralSum = integralSum + (error * timer.seconds());

            double power = (kp * error) + (ki * integralSum) + (kd * derivate);

            //Calculo da posição de correção do motor da Turret
            double range = 288.0 / 360.0;
            double ticks = tx * range;
            int target = (int) (currentPosition + ticks);

            //Seta a posição do motor da Turret pra se alinhas noo goal
            corehex.setTargetPosition(target);
            corehex.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Se a limelight ler a tag ele se alinha
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                if (fr.getFiducialId() == id) {
                    if (Math.abs(tx) > 2.5) {
                        corehex.setPower(power);
                    } else {
                        corehex.setPower(0);
                    }
                }
            }

            lastError = error;
            timer.reset();
        }

        //Se não o motor para
        else {
            corehex.setPower(0);
        }
    }

    //Atira os artefatos no goal
    public void shoot(boolean red){
        //Captura os resultados da April Tag
        LLResult resultShoot = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducialResults = resultShoot.getFiducialResults();

        //Se a red for verdadeiro, o id vai ser 24. Senão, vai ser 20
        int id = red ? 24 : 20;

        //Parâmetros da fóruma de velocidade :
        // a == coeficiente de ganho proporcional à distância
        // b == valor base (offset) da rotação mínima
        // MAX_RPM = limite máximo de rotação do flywheel
        double a = 20.0;
        double b = 1500.0;
        double MAX_RPM = 6000;

        //Calcula a distancia da limelight até o goal.
        double distance = 181.9994 / Math.sqrt(resultShoot.getTa());

       //Aplica uma fórmula linear para definir o RPM alvo com base na distância.
        // Se o resultado ultrapassar o máximo permitido (MAX_RPM), ele é limitado.
        double targetRPM = a * distance + b;
        targetRPM = Math.min(targetRPM, MAX_RPM);

        //Converte o RPM em velocidade do motor em ticks por segundo
        double TICKS_PER_REV = 28;
        double targetVelocity = (targetRPM * TICKS_PER_REV) / 60.00;

        //Apenas é acionado quando o id encontrado for igual ao id da cor da aliança
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            if (fr.getFiducialId() == id) {
                flywheel.setVelocity(targetVelocity);
            }
        }

    }

    //Pega os artefatos
    public void intake(boolean run){
        if(run){
            intake.setPower(1);
        }else{
            intake.setPower(0);
        }
    }

    //Transfere os artefatos do intake para a Turret
    public void transfer(boolean transferir){
        if(transferir = true){
            transfer.setPower(1);
        }
        if(transferir = false){
            transfer.setPower(0);
        }
    }
}
