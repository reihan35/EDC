package algorithms;

import java.io.File;
import java.time.Duration;
import java.time.Instant;

public class Test {

	public static void main(String[] args) {
		final File folder = new File("TAILLE_5000");
		int somme_taille = 0;
		long somme_tps = 0;
    	for (File file : folder.listFiles()) {
    		//System.out.println(file.getName());
    		Instant start = Instant.now();
    		int res = DefaultTeam.calculConnectedDominatingSet(folder.getName()+ "//" +file.getName(),25);
            Instant finish = Instant.now();
    		somme_taille = somme_taille + res;
    		long timeElapsed = Duration.between(start, finish).toMillis(); 
    		somme_tps = somme_tps + timeElapsed;
    	}
    	System.out.println("moyenne taille " + somme_taille/100);
    	System.out.println("moyenne tps " + somme_tps/100);
	}

}
