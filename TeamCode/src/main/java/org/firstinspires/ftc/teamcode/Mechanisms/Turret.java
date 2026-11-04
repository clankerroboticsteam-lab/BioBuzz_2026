package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.jvm.Pool;

import org.firstinspires.ftc.vision.apriltag.AprilTagCanvasAnnotator;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class Turret {
    private DcMotorEx turret;

    private double kP = 0.0001;
    private double kD = 0;

    private double goalX = 0;

    private double lastError = 0;
    private double angleTolerance = 0.2;

    //tune this please
    private final double Max_POWER = 0.6;
    private double power = 0;
    private final ElapsedTime timer = new ElapsedTime();

    private HardwareMap hwMap;

    private void init(HardwareMap hwMap) {
        turret = hwMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setkP(double newKP) {
        kP = newKP;
    }

    public double getkP(){
        return kP;
    }
    public void setkD(double newKD) {
        kP = newKD;
    }
    public double getkD(){
        return kD;
    }

    public void resetTimer(){
    timer.reset();

    }

    public void update(LLResult curID){}

}
