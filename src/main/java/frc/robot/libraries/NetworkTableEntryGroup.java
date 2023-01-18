package frc.robot.libraries;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class NetworkTableEntryGroup {
    public GenericEntry[] Entries = new GenericEntry[4];
    
    public NetworkTableEntryGroup(ShuffleboardTab tab, String title, double defaultSet) {
      double[] defaults = {defaultSet, defaultSet, defaultSet, defaultSet};
      build(tab, title, defaults);
    }
    public NetworkTableEntryGroup(ShuffleboardTab tab, String title, double[] defaultSet) {
        build(tab, title, defaultSet);
    }

    void build(ShuffleboardTab tab, String title, double[] defaultSet) {
      for (int i = 0; i < 4; i++) {Entries[i] = tab.add(
        aPosMask(i) + " " + title,  defaultSet[i]
      ).getEntry();}
    }

    String aPosMask(int i) {
        switch (i) {
            case 0: return "L1"; case 1: return "R1";
            case 2: return "L2"; case 3: return "R2";
            default: return "<InvalidInput>";
        }
    }
  }