package frc.robot.tools;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class PneumaticsControl {
    
    //private final PneumaticsControlModule hub = new PneumaticsControlModule();
    private final PneumaticsControlModule pcm = new PneumaticsControlModule();
    private final Compressor compressor = pcm.makeCompressor();
    private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 5, 6);
    // private final DoubleSolenoid climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);

    public void setIntakeUp() {
        intakeSolenoid.set(Value.kReverse);
    }

    public void setIntakeDown() {
        intakeSolenoid.set(Value.kForward);
    }

    public double getCompressorCurrent(){
        return pcm.getCompressorCurrent();
    }

    
    // public void engageClimberBrake() {
    //     climberSolenoid.set(Value.kReverse);
    // }

    // public void releaseClimberBrake() {
    //     climberSolenoid.set(Value.kForward);
    // }

}
