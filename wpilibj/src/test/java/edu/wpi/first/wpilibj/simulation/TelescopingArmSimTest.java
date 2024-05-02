package edu.wpi.first.wpilibj.simulation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import org.junit.jupiter.api.Test;

class TelescopingArmSimTest {
  @Test
  void testLimits() {
    TelescopingArmSim telescopingArmSim =
        new TelescopingArmSim(
            DCMotor.getNEO(2),
            DCMotor.getNEO(1),
            0.02,
            -Math.PI / 6,
            Math.PI,
            0,
            1,
            3,
            3,
            0.5,
            0.5,
            8,
            8,
            0,
            0.1);
    assertFalse(telescopingArmSim.hasHitLowerAngleLimit());
    assertFalse(telescopingArmSim.hasHitLowerExtensionLimit());
    assertTrue(telescopingArmSim.wouldHitUpperExtensionLimit(2));
    telescopingArmSim.setInputVoltages(VecBuilder.fill(-12, -12));
    for (int i = 0; i < 100; i++) {
      telescopingArmSim.update(0.02);
    }
    assertEquals(-Math.PI / 6, telescopingArmSim.getPivotAngleRads(), 0.01);
    assertEquals(0, telescopingArmSim.getExtensionMeters(), 0.01);
    telescopingArmSim.setState(0, 0, 0, 0);
    telescopingArmSim.setInputVoltages(VecBuilder.fill(12, 12));
    for (int i = 0; i < 200; i++) {
      telescopingArmSim.update(0.02);
    }
    assertEquals(Math.PI, telescopingArmSim.getPivotAngleRads(), 0.01);
    assertEquals(1, telescopingArmSim.getExtensionMeters(), 0.01);
  }
}
