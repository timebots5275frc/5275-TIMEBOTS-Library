package frc.robot.CustomTypes;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class PID {
    public double p,i,d,kS,kV,kA,kG,iz;
    public PID(double p, double i, double d, double kS) {this.p=p; this.i=i; this.kS=kS; this.d=d;}
    public PID(double p, double i, double d, double iz, double kS) {this.p=p; this.i=i; this.d=d; this.iz=iz; this.kS=kS;}
    public PID(double p, double i, double d, double kS, double kV, double kA, double kG){this.p=p; this.i=i; this.d=d; this.kS=kS; this.kV=kV; this.kA=kA; this.kG=kG;}
    public PID() {this(0,0,0,0);}
    public PID(double p, double i, double d) {this(p,i,d,0);}
    public PID(PID cpy) {this(cpy.p, cpy.i, cpy.d);}

    public final IdleMode _imode_default = IdleMode.kBrake;
    public final PersistMode  _pmode_default = PersistMode.kNoPersistParameters;
    public final ResetMode _rmode_default = ResetMode.kResetSafeParameters;
    private IdleMode _imode;
    private PersistMode _pmode;
    private ResetMode _rmode;
    private int stallLimit = 20; //Defaults
    private int freeLimit = 20;  //Defaults



    // ---------- Public API: same name, return type follows the motor type ----------

    public SparkMaxConfig setPID(SparkMax motor) {
        return setPID(motor, null, null, null);
    }
    public SparkMaxConfig setPID(SparkMax motor, IdleMode im) {
        return setPID(motor, null, null, im);
    }
    public SparkMaxConfig setPID(SparkMax motor, ResetMode rm, PersistMode pm) {
        return setPID(motor, rm, pm, null);
    }
    public SparkMaxConfig setPID(SparkMax motor, ResetMode rm, PersistMode pm, IdleMode im) {
        return buildAndApply(new SparkMaxConfig(), motor, rm, pm, im);
    }

    public SparkFlexConfig setPID(SparkFlex motor) {
        return setPID(motor, null, null, null);
    }
    public SparkFlexConfig setPID(SparkFlex motor, IdleMode im) {
        return setPID(motor, null, null, im);
    }
    public SparkFlexConfig setPID(SparkFlex motor, ResetMode rm, PersistMode pm) {
        return setPID(motor, rm, pm, null);
    }
    public SparkFlexConfig setPID(SparkFlex motor, ResetMode rm, PersistMode pm, IdleMode im) {
        return buildAndApply(new SparkFlexConfig(), motor, rm, pm, im);
    }


    private <C extends SparkBaseConfig> C buildAndApply(C sc, SparkBase motor, ResetMode rm, PersistMode pm, IdleMode im) {
        ClosedLoopConfig c = new ClosedLoopConfig();
        c.pidf(p, i, d, kS);
        c.iZone(iz);

        FeedForwardConfig fc = new FeedForwardConfig();
        fc.kV(kV);
        fc.kS(kS);
        fc.kA(kA);
        fc.kG(kG);
        c.feedForward.apply(fc);

        sc.apply(c);
        sc.smartCurrentLimit(stallLimit, freeLimit);
        sc.idleMode(im != null ? im : _imode_default);

        motor.configure(sc,rm != null ? rm : _rmode_default,pm != null ? pm : _pmode_default);

        return sc;
    }

    public void setFreeLimit(int freeLimit) {
      this.freeLimit = freeLimit;
    }
     public void setStallLimit(int stallLimit) {
      this.stallLimit = stallLimit;
    }
}