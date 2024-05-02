// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.wpilibj.simulation;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;

/** Represents a simulated single jointed arm mechanism. */
public class TelescopingArmSim {
  private final DCMotor m_pivotGearbox;
  private final DCMotor m_extensionGearbox;

  private final double m_drumRadiusMeters;

  private final double m_minAngle;

  private final double m_maxAngle;

  private final double m_minExtension;

  private final double m_maxExtension;

  private final double m_moiKgMetersSquared0; // inertia of the inner telescope
  private final double m_moiKgMetersSquared1; // inertia of the telescoping segment
  private final double m_centerOfMassMeters0; // CoM distance from the pivot for seg 0
  private final double m_centerOfMassMeters1; // CoM distance from pivot for seg 1, when x=0
  private final double m_massKg0; // mass of 0
  private final double m_massKg1; // mass of 1

  private Matrix<N4, N1> m_x; // θ,x,ω,ẋ

  private Matrix<N2, N1> m_u;

  /**
   * Creates a simulated telescoping arm mechanism.
   *
   * @param pivotGearbox The motor for the pivot
   * @param extensionGearbox The motor for the telescope
   * @param drumRadiusMeters The radius of the telescope drum
   * @param minAngle The minimum angle of the pivot in radians
   * @param maxAngle The maximum angle of the pivot in radians
   * @param minExtension The minimum extension of the telescope in meters
   * @param maxExtension The maximum extension of the telescope in meters
   * @param moiKgMetersSquared0 The moment of inertia of the fixed segment of the telescope
   * @param moiKgMetersSquared1 The moment of inertia of the telescoping segment of the telescope
   * @param centerOfMassMeters0 The distance from the pivot of the center of mass of the fixed
   *     segment of the telescope
   * @param centerOfMassMeters1 The distance from the pivot of the center of mass of the telescoping
   *     segment of the telescope, when its extension is zero
   * @param massKg0 The mass of the fixed segment of the telescope
   * @param massKg1 The mass of the telescoping segment of the telescope
   * @param startingAngleRads The starting angle of the pivot
   * @param startingExtensionMeters The starting extension
   */
  public TelescopingArmSim(
      DCMotor pivotGearbox,
      DCMotor extensionGearbox,
      double drumRadiusMeters,
      double minAngle,
      double maxAngle,
      double minExtension,
      double maxExtension,
      double moiKgMetersSquared0,
      double moiKgMetersSquared1,
      double centerOfMassMeters0,
      double centerOfMassMeters1,
      double massKg0,
      double massKg1,
      double startingAngleRads,
      double startingExtensionMeters) {
    m_pivotGearbox = pivotGearbox;
    m_extensionGearbox = extensionGearbox;
    m_drumRadiusMeters = drumRadiusMeters;
    m_minAngle = minAngle;
    m_maxAngle = maxAngle;
    m_minExtension = minExtension;
    m_maxExtension = maxExtension;
    m_moiKgMetersSquared0 = moiKgMetersSquared0;
    m_moiKgMetersSquared1 = moiKgMetersSquared1;
    m_centerOfMassMeters0 = centerOfMassMeters0;
    m_centerOfMassMeters1 = centerOfMassMeters1;
    m_massKg0 = massKg0;
    m_massKg1 = massKg1;
    setState(startingAngleRads, startingExtensionMeters, 0, 0);
  }

  public void setState(Matrix<N4, N1> state) {
    m_x = state;
  }

  /**
   * Sets the state of the simulated telescoping arm.
   *
   * @param pivotAngleRadians The angle of the pivot in radians
   * @param extensionMeters The length of the extension in meters
   * @param pivotVelocityRadPerSec The angular velocity of the pivot in radians per second
   * @param extensionVelocityMetersPerSec The velocity of the extension in meters per second
   */
  public void setState(
      double pivotAngleRadians,
      double extensionMeters,
      double pivotVelocityRadPerSec,
      double extensionVelocityMetersPerSec) {
    setState(
        VecBuilder.fill(
            pivotAngleRadians,
            extensionMeters,
            pivotVelocityRadPerSec,
            extensionVelocityMetersPerSec));
  }

  public boolean wouldHitLowerAngleLimit(double currentAngleRads) {
    return currentAngleRads <= this.m_minAngle;
  }

  public boolean wouldHitUpperAngleLimit(double currentAngleRads) {
    return currentAngleRads >= this.m_maxAngle;
  }

  public boolean hasHitLowerAngleLimit() {
    return wouldHitLowerAngleLimit(getPivotAngleRads());
  }

  public boolean hasHitUpperAngleLimit() {
    return wouldHitUpperAngleLimit(getPivotAngleRads());
  }

  public boolean wouldHitLowerExtensionLimit(double currentExtensionMeters) {
    return currentExtensionMeters <= this.m_minExtension;
  }

  public boolean wouldHitUpperExtensionLimit(double currentExtensionMeters) {
    return currentExtensionMeters >= this.m_maxExtension;
  }

  public boolean hasHitLowerExtensionLimit() {
    return wouldHitLowerAngleLimit(getExtensionMeters());
  }

  public boolean hasHitUpperExtensionLimit() {
    return wouldHitUpperAngleLimit(getExtensionMeters());
  }

  public double getExtensionMeters() {
    return m_x.get(1, 0);
  }

  public double getExtensionVelocityMetersPerSec() {
    return m_x.get(3, 0);
  }

  public double getPivotAngleRads() {
    return m_x.get(0, 0);
  }

  public double getPivotVelocityRadPerSec() {
    return m_x.get(2, 0);
  }

  /**
   * Gets the current draw of the motors.
   *
   * @return The current draw of the pivot and extension motors
   */
  public Matrix<N2, N1> getCurrentDrawsAmps() {
    return VecBuilder.fill(
        m_pivotGearbox.getCurrent(m_x.get(2, 0), m_u.get(0, 0)),
        m_extensionGearbox.getCurrent(m_x.get(3, 0) / m_drumRadiusMeters, m_u.get(1, 0)));
  }

  public double getPivotCurrentDrawAmps() {
    return getCurrentDrawsAmps().get(0, 0);
  }

  public double getExtensionCurrentDrawAmps() {
    return getCurrentDrawsAmps().get(1, 0);
  }

  public void setInputVoltages(Matrix<N2, N1> voltages) {
    m_u = voltages;
  }

  public void setInputPivotVoltage(double volts) {
    m_u.set(0, 0, volts);
  }

  public void setInputExtensionVoltage(double volts) {
    m_u.set(1, 0, volts);
  }

  public Matrix<N2, N1> getVoltages() {
    return m_u;
  }

  public double getPivotVoltage() {
    return getVoltages().get(0, 0);
  }

  public double getExtensionVoltage() {
    return getVoltages().get(1, 0);
  }

  /**
   * Updates the simulation.
   *
   * @param dtSeconds The time between updates
   */
  public void update(double dtSeconds) {
    /* Lagrangian mechanics for Telescoping arm
     * J_0: inertia of the inner telescope
     * J_1: inertia of the telescoping segment
     * r_0: CoM distance from the pivot for seg 0
     * r_1: CoM distance from pivot for seg 1, when x=0
     * m_0: mass of 0
     * m_1: mass of 1
     *
     * T=(0.5*(J_0+J_1)*ω²)+(0.5*m_0*r_0²*ω²)+(0.5*m_1*(ẋ²+((r_1+x)*ω)²))
     * V=g*cos(θ)*(m_0*r_0+m_1*(x+r_1))
     * L=T-V
     *
     * τ_i=d/dt(dL/dẋ_i) - dL/dx_i
     *
     * dL/dẋ=m_1*ẋ
     * d/dt(dL/dẋ)=m_1*ẍ
     * dL/dx=m_1*((r_1+x)*ω²-g*cos(θ))
     * F=m_1*(ẍ+g*cos(θ)-(r_1+x)*ω²)
     * ẍ=(F/m_1)+(r_1+x)*ω²-g*cos(θ)
     *
     *
     * dL/dω= ω*(J_0+J_1+m_0*r_0²+m_1*(r_1+x)²)
     * d/dt(dL/dω)=α*(J_0+J_1+m_0*r_0²+m_1*(r_1+x)²)+ω*(2*m_1*ẋ*(r_1+x))
     * dL/dθ=g*sin(θ)*(m_0*r_0+m_1*(x+r_1))
     * τ=α*(J_0+J_1+m_0*r_0²+m_1*(r_1+x)²)+ω*(2*m_1*ẋ*(r_1+x))-g*sin(θ)*(m_0*r_0+m_1*(x+r_1))
     * α=(τ+g*sin(θ)*(m_0*r_0+m_1*(x+r_1))-ω*(2*m_1*ẋ*(r_1+x)))/(J_0+J_1+m_0*r_0²+m_1*(r_1+x)²)
     * */

    Matrix<N4, N1> updatedX =
        NumericalIntegration.rkdp(
            (Matrix<N4, N1> x, Matrix<N2, N1> u) -> {
              double pivotTorque =
                  m_pivotGearbox.getTorque(m_pivotGearbox.getCurrent(x.get(2, 0), u.get(0, 0)));
              double extensionForce =
                  m_extensionGearbox.getTorque(
                          m_extensionGearbox.getCurrent(
                              x.get(2, 0) / m_drumRadiusMeters, u.get(0, 0)))
                      / m_drumRadiusMeters;

              double pivotAcceleration =
                  (pivotTorque
                          + 9.8
                              * Math.sin(x.get(0, 0))
                              * (m_massKg0 * m_centerOfMassMeters0
                                  + m_massKg1 * (x.get(1, 0) + m_centerOfMassMeters1))
                          - x.get(2, 0)
                              * (2
                                  * m_massKg1
                                  * x.get(3, 0)
                                  * (m_centerOfMassMeters1 + x.get(1, 0))))
                      / (m_moiKgMetersSquared0
                          + m_moiKgMetersSquared1
                          + m_massKg0 * Math.pow(m_centerOfMassMeters0, 2)
                          + m_massKg1 * Math.pow(m_centerOfMassMeters1 + x.get(1, 0), 2));

              double extensionAcceleration =
                  (extensionForce / m_massKg1)
                      + (m_centerOfMassMeters1 + x.get(1, 0)) * Math.pow(x.get(2, 0), 2)
                      - 9.8 * Math.cos(x.get(0, 0));

              Matrix<N4, N1> xdot =
                  VecBuilder.fill(
                      x.get(2, 0), x.get(3, 0), pivotAcceleration, extensionAcceleration);
              return xdot;
            },
            m_x,
            m_u,
            dtSeconds);
    m_x = updatedX;
  }
}
