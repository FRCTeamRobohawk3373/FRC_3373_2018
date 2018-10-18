package org.usfirst.frc.team3373.robot;

import edu.wpi.first.wpilibj.Relay;

public class Grabber {
	private boolean isCubeIn;
	SupremeTalon grab1;
	SupremeTalon grab2;
	int spikeCount;
	public Grabber(int port1, int port2){
		isCubeIn = false;
		grab1 = new SupremeTalon(port1);
		grab2 = new SupremeTalon(port2);
		spikeCount = 0;
	}
	public void importCube(){ //put cube in grabber
		isCubeIn = true;
		grab2.set(1);
		grab1.set(-1);
	}
	public void exportCube(){ //remove cube from grabber
		isCubeIn = false;
		grab2.set(-1);
		grab1.set(1);
	}
	public void set(double power){
		grab1.set(-power);
		grab2.set(power);
	}
/*	public void keepCube(){ // keep cube in grabber
		grab1.accelerate(.2,.05,false);
		grab2.accelerate(.2,.05,false);
	}*/
	public void idle(){ // grabber wait for cube input
		grab1.set(0);
		grab2.set(0);
	}
	public boolean hasCube(){
		if(((grab1.getOutputCurrent()+grab2.getOutputCurrent())/2)>4.5){
			spikeCount++;
		}
		else{
			spikeCount = 0;
		}
		if(spikeCount > 25){
			isCubeIn = true;
		}
		return isCubeIn;
	}
	public void resetHasCube(){
		isCubeIn = false;
	}
}
