package frc.robot.CustomTypes;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MathConstants;

/**
 * Holds closed loop gains ONLY (current limits / idle mode live in MotorData).
 *
 * Live tuning usage (in a subsystem):
 *   constructor:  MY_PID.tunable("Drive", leftLeaderMotor, rightLeaderMotor);
 *   periodic():   MY_PID.updateLive();
 * Then in Test mode, edit  Tuning/Drive/P  etc. on the dashboard.
 */
public class PID {
    public double p, i, d, kS, kV, kA, kG, iz;
    // build constructors for all combinations of parameters, with defaults for the rest
    public PID() {this(0,0,0,0);}
    public PID(double p, double i, double d) {this(p,i,d,0);}
    public PID(double p, double i, double d, double kS) {this.p=p; this.i=i; this.d=d; this.kS=kS;}
    public PID(double p, double i, double d, double iz, double kS) {this.p=p; this.i=i; this.d=d; this.iz=iz; this.kS=kS;}
    public PID(double p, double i, double d, double kS, double kV, double kA, double kG) {this.p=p; this.i=i; this.d=d; this.kS=kS; this.kV=kV; this.kA=kA; this.kG=kG;}
    /** copy from other pid */
    public PID(PID c) {this.p=c.p; this.i=c.i; this.d=c.d; this.kS=c.kS; this.kV=c.kV; this.kA=c.kA; this.kG=c.kG; this.iz=c.iz;}

    /** Writes these gains into a config. Does not touch any motor.
     * @param sc the config to apply the gains to
     * @return the config with the gains applied
     */
    public <C extends SparkBaseConfig> C applyTo(C sc) {
        ClosedLoopConfig c = new ClosedLoopConfig();
        c.pid(p, i, d);
        c.iZone(iz);

        FeedForwardConfig fc = new FeedForwardConfig();
        fc.kS(kS);
        fc.kV(kV);
        fc.kA(kA);
        fc.kG(kG);
        c.feedForward.apply(fc);

        sc.apply(c);
        return sc;
    }

    //#region LIVE PID TUNING
    private String tuneName = null;
    private final List<SparkBase> tuneMotors = new ArrayList<>();

    /**
     * Call ONCE in the subsystem constructor, after the motors exist.
     * Publishes the current gains to the dashboard and remembers which motors to update.
     * Pass leaders only; followers copy the leader.
     */
    public PID tunable(String name, SparkBase... motors) {
        tuneName = name;
        for (SparkBase m : motors) tuneMotors.add(m);
        publish();
        return this;
    }

    private String key(String field) {return "Tuning/" + tuneName + "/" + field;}

    /**
     * publish ONCE
     */
    private void publish() {
        SmartDashboard.putNumber(key("P"), p);
        SmartDashboard.putNumber(key("I"), i);
        SmartDashboard.putNumber(key("D"), d);
        SmartDashboard.putNumber(key("kS"), kS);
        SmartDashboard.putNumber(key("kV"), kV);
        SmartDashboard.putNumber(key("IZone"), iz);
    }

    /**
     * Returns true if the two values are different enough to be considered changed.
     * @param a
     * @param b
     * @return
     */
    private static boolean changed(double a, double b) {return Math.abs(a - b) >= MathConstants.EPSILON;}

    /** Call every periodic. Does nothing unless in Test mode and a value was edited. tunable() must have been called before */
    @SuppressWarnings("removal")
    public void updateLive() {
        if (tuneName == null || !RobotState.isTest()) return; // do nothing if not in test mode or if tunable() was never called

        //get new values
        double np  = SmartDashboard.getNumber(key("P"),  p);
        double ni  = SmartDashboard.getNumber(key("I"),  i);
        double nd  = SmartDashboard.getNumber(key("D"),  d);
        double nkS = SmartDashboard.getNumber(key("kS"), kS);
        double nkV = SmartDashboard.getNumber(key("kV"), kV);
        double niz = SmartDashboard.getNumber(key("IZone"), iz);

        //check for change
        if (!changed(np, p) && !changed(ni, i) && !changed(nd, d) && !changed(nkS, kS) && !changed(nkV, kV) && !changed(niz, iz)) return; // if no change, do nothing

        //update values
        p = np; i = ni; d = nd; kS = nkS; kV = nkV; iz = niz;

        // Re-apply ONLY the closed loop settings to the motors that already exist.
        // No new motor objects, no parameter reset, no flash write.
        for (SparkBase m : tuneMotors) {
            SparkBaseConfig cfg = (m instanceof SparkFlex) ? new SparkFlexConfig() : new SparkMaxConfig(); // if it is a sparkflex, use sparkflexconfig, else use sparkmaxconfig 
            applyTo(cfg);
            m.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }
    //#endregion LIVE PID TUNING
}