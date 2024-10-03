package team1403.lib.util;

import monologue.Logged;

//Allows for easy AdvantageKit Interop in the future
public interface CougarLogged extends Logged {
    
    @Override
    public default String getFullPath() {
      return "/AdvantageKit/RealOutputs";
   }
}
