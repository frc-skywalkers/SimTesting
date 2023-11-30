package frc.robot.subsystems.elevator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.robot.Constants.ElevatorConstants;

//almost exactly the same as wpilib elevatorsim only instead of using 9.8 for gravity it uses 9.8sin(theta)
public class TiltedElevatorSim extends LinearSystemSim<N2, N1, N1>{
    //elevator gearbox
    private final DCMotor m_gearbox;
    //gearing between motors and output
    private final double m_gearing;
    //radius of drum spool is wrapped around
    private final double m_drumRadius;
    //min allowable height
    private final double m_minHeight;
    //max allowable height
    private final double m_maxHeight;
    //simulate gravity or no
    private final boolean m_simulateGravity;

    private double m_angleFromHorizontal = ElevatorConstants.tiltAngle;

    public TiltedElevatorSim(
        LinearSystem<N2, N1, N1> plant,
        DCMotor gearbox,
        double gearing,
        double drumRadiusMeters,
        double minHeightMeters,
        double maxHeightMeters,
        boolean simulateGravity) {
      this(
        plant,
        gearbox,
        gearing,
        drumRadiusMeters,
        minHeightMeters,
        maxHeightMeters,
        simulateGravity,
        null
        );
    }

    public TiltedElevatorSim(
        LinearSystem<N2, N1, N1> plant,
        DCMotor gearbox,
        double gearing,
        double drumRadiusMeters,
        double minHeightMeters,
        double maxHeightMeters,
        boolean simulateGravity,
        Matrix<N1, N1> measurementStdDevs) {
      super(plant, measurementStdDevs);
      m_gearbox = gearbox;
      m_gearing = gearing;
      m_drumRadius = drumRadiusMeters;
      m_minHeight = minHeightMeters;
      m_maxHeight = maxHeightMeters;
      m_simulateGravity = simulateGravity;
    }
    
    public TiltedElevatorSim(
        DCMotor gearbox,
        double gearing,
        double carriageMassKg,
        double drumRadiusMeters,
        double minHeightMeters,
        double maxHeightMeters,
        boolean simulateGravity) {
    this(
        gearbox,
        gearing,
        carriageMassKg,
        drumRadiusMeters,
        minHeightMeters,
        maxHeightMeters,
        simulateGravity,
        null);
    }
    
    public TiltedElevatorSim(
        DCMotor gearbox,
        double gearing,
        double carriageMassKg,
        double drumRadiusMeters,
        double minHeightMeters,
        double maxHeightMeters,
        boolean simulateGravity,
        Matrix<N1, N1> measurementStdDevs) {
      super(
        LinearSystemId.createElevatorSystem(gearbox, carriageMassKg, drumRadiusMeters, gearing),
        measurementStdDevs);
      m_gearbox = gearbox;
      m_gearing = gearing;
      m_drumRadius = drumRadiusMeters;
      m_minHeight = minHeightMeters;
      m_maxHeight = maxHeightMeters;
      m_simulateGravity = simulateGravity;
    }
    
    public boolean wouldHitLowerLimit(double elevatorHeightMeters) {
        return elevatorHeightMeters < this.m_minHeight;
    }

    public boolean wouldHitUpperLimit(double elevatorHeightMeters) {
        return elevatorHeightMeters > this.m_maxHeight;
    }

    public boolean hasHitUpperLimit() {
        return wouldHitUpperLimit(getPositionMeters());
    }

    public double getPositionMeters() {
        return getOutput(0);
    }

    public double getVelocityMetersPerSecond(){
        return m_x.get(1, 0);
    }

    @Override
    public double getCurrentDrawAmps() {
        double motorVelocityRadPerSec = getVelocityMetersPerSecond() / m_drumRadius * m_gearing;
        var appliedVoltage = m_u.get(0, 0);
        return m_gearbox.getCurrent(motorVelocityRadPerSec, appliedVoltage) * Math.signum(appliedVoltage);
    }

    public void setInputVoltage(double volts) {
        setInput(volts);
    }

    public void setAngleFromHorizontal(double angle) {
        m_angleFromHorizontal = angle;
    }

    @Override
    protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
        var updatedXhat =
            NumericalIntegration.rkdp(
                (x, _u) -> {
                    Matrix<N2, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(_u));
                    if (m_simulateGravity) {
                        xdot = xdot.plus(VecBuilder.fill(0, -9.8 * Math.sin(m_angleFromHorizontal)));
                    }
                    return xdot;
                },
                currentXhat,
                u,
                dtSeconds);
        if (wouldHitLowerLimit(updatedXhat.get(0, 0))) {
            return VecBuilder.fill(m_minHeight, 0);
        }
        if (wouldHitUpperLimit(updatedXhat.get(0, 0))) {
            return VecBuilder.fill(m_maxHeight, 0);
        }
        return updatedXhat;
    }

}