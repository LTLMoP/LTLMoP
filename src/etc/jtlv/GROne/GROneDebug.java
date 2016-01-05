import net.sf.javabdd.BDD;
import java.io.*;
import net.sf.javabdd.BDDVarSet;
import net.sf.javabdd.BDD.BDDIterator;
import edu.wis.jtlv.env.Env;
import edu.wis.jtlv.env.module.SMVModule;
import edu.wis.jtlv.env.spec.Spec;
import java.io.File;
import java.io.PrintStream;
import java.io.OutputStream;
import java.io.FileWriter;
import java.io.BufferedWriter;
import edu.wis.jtlv.lib.FixPoint;
import edu.wis.jtlv.old_lib.games.GameException;
import edu.wis.jtlv.env.spec.SpecBDD;
import edu.wis.jtlv.lib.AlgRunnerThread;
import edu.wis.jtlv.lib.AlgResultI;
import edu.wis.jtlv.lib.mc.tl.LTLModelCheckAlg;

// Class containing methods for performing unsatisfiability and unrealizability checks on a specification 
// Spec consists of environment and system modules (env, sys)

public class GROneDebug {
	
	
	/**
	 * @param args
	 * @throws Exception
	 */
	public static void main(String[] args) throws Exception {
		// uncomment to use a C BDD package
		//System.setProperty("bdd", "buddy");

		GROneParser.just_initial = true;
		// GRParser.just_safety = true;

        // Check that we have enough arguments
        // This class takes the same arguments as GROneMain.
	if (args.length < 2) {
            System.err.println("Usage: java GROneDebug [smv_file] [ltl_file]");
            System.exit(1);
        }
	
	boolean refine = false;
	if (args.length == 3) {
            refine =  Boolean.parseBoolean(args[2]);
        }
	
	Env.loadModule(args[0]);
        Spec[] spcs = Env.loadSpecFile(args[1]);

        // Figure out the name of our output file by stripping the spec filename extension and adding .aut
        String out_filename = args[1].replaceAll("\\.[^\\.]+$",".aut");

		// constructing the environment module.
		SMVModule env = (SMVModule) Env.getModule("main.e");
		Spec[] env_conjuncts = GROneParser.parseConjuncts(spcs[0]);
		GROneParser.addReactiveBehavior(env, env_conjuncts);
		// GRParser.addPureReactiveBehavior(env, env_conjuncts);

		// constructing the system module.
		SMVModule sys = (SMVModule) Env.getModule("main.s");
		Spec[] sys_conjuncts = GROneParser.parseConjuncts(spcs[1]);
		GROneParser.addReactiveBehavior(sys, sys_conjuncts);
		//GROneParser.addPureReactiveBehavior(sys, sys_conjuncts);		
		
		
		//Prints the results of analyzing the specification. 
		if (!refine) {
			System.out.println(analyze(env, sys));
		} else {
			System.out.println(refine(env, sys, args));
		}
	}

	
	public static String analyze(SMVModule env, SMVModule sys) {

		  
		int explainSys=0, explainEnv = 0; //keep track of explanations to avoid redundancy
		
		BDD sys_init = sys.initial();
		BDD env_init = env.initial();
		BDD all_init = sys_init.and(env_init);	 
		 
		//BDDs used to identify cases of unsynthesizability
		BDD envUnreal, sysUnreal; 
		BDD envUnsat, sysUnsat;

		FixPoint<BDD> iter;
		
		String debugInfo = ""; //will eventually contain all debugging statements to be displayed
		
			
		 //Fixpoint computation to determine reachability of environment deadlock states
		 BDD cox = Env.FALSE();
		 for (iter = new FixPoint<BDD>(); iter.advance(cox);) {			 
			 cox = cox.or(env.yieldStates(sys, cox));
		 }
		 envUnreal = cox.id().and(all_init);

		 //Fixpoint computation to determine reachability of system deadlock states
		 cox = Env.FALSE();
		 for (iter = new FixPoint<BDD>(); iter.advance(cox);) {
			 cox = cox.or((env.yieldStates(sys, cox.not())).not());			 
		 }		 
		 sysUnreal = cox.id().and(all_init);
		 BDD unrealCause = (env.yieldStates(sys, Env.TRUE()).not()).exist(sys.modulePrimeVars());	
		 
		 //Some simple satisfiability tests
		 if (sys.initial().isZero()) {
			 debugInfo += "System initial condition is unsatisfiable." + "\n";
			 explainSys = 1;
		 
		 } else if (((sys.trans())).isZero()) {
			 debugInfo += "System transition relation is unsatisfiable." + "\n";
			 explainSys = 1;
		 } 
		  
		 if (env.initial().isZero()) {
			  debugInfo += "Environment initial condition is unsatisfiable." + "\n";
			  explainEnv = 1;	 
		 } else if (((env.trans())).isZero()) {
			  debugInfo += "Environment transition relation is unsatisfiable." + "\n";
			  explainEnv = 1;
		 } 
		 
		 //create a GR(1) game using the specified modules for the rest of the checks
		 GROneGame g = null;		 
		 try { 
			  	g = new GROneGame(env,sys, sys.justiceNum(), env.justiceNum());		
		  }	catch (Exception e){//Catch exception if any
					System.err.println("Error: " + e.getMessage());
		  }

		 BDD counter_example = g.envWinningStates().and(all_init);
		 
		 boolean falsifyEnv = false;
		  
		 if (counter_example.isZero()) {
			 //falsifyEnv tells us whether an environment liveness was falsified in the system's winning strategy		
		    PrintStream orig_out = System.out;	
		 	PrintStream ignore = new PrintStream(new NullOutputStream());
		 	System.setOut(ignore);
			falsifyEnv = g.printWinningStrategy(all_init);
			System.setOut(orig_out); // restore STDOUT
		 }

		  
		  try { 
			  if (explainSys == 0 && !counter_example.isZero()) {
				//checking for multi-step unsatisfiability between sys transitions/safety and initial condition					 
				//since the second argument is false, we are looking for deadlock 	
				if (g.calculate_counterstrategy(counter_example, false, false)) { 			 
					 debugInfo += "System initial condition inconsistent with transition relation." + "\n";
					 explainSys = 1;
				 }
			  } else if (explainEnv == 0 && counter_example.isZero()) {		 
				//checking for multi-step unsatisfiability between env transitions/safety and initial condition
				//since the first argument is 1, we only allow system transitions of type \rho_1
				//so we are looking for sequences of moves that take us to deadlock only
				  if (g.calculate_strategy(1, g.getSysPlayer().initial().and(g.getEnvPlayer().initial()), false)) { 
						 debugInfo += "Environment initial condition inconsistent with transition relation. " + "\n";
						 explainEnv = 1;
				  }
			  }
		  }	catch (Exception e){//Catch exception if any
				System.err.println("Error: " + e.getMessage());
	  }  

		  if (explainSys ==0 && !sysUnreal.equals(Env.FALSE())) {
				 debugInfo += "System is unrealizable because the environment can force a safety violation."+unrealCause+ "\n"; //TRUE if unsat
		  		 explainSys = 1;
		  }
		  
		  if (explainEnv ==0 && !envUnreal.equals(Env.FALSE())) {		 
			  	debugInfo += "Environment is unrealizable because the system can force a safety violation."+ "\n"; //TRUE if unsat
		  		explainEnv = 1;
		  }
		  
		  
		// check for unsatisfiability or unrealizability of the justice/liveness (vs. safety) conditions
		try{
			    debugInfo += justiceChecks(env,sys,explainSys,explainEnv, falsifyEnv);
		}catch (Exception e){//Catch exception if any
			      System.err.println("Error: " + e.getMessage());
		}
		
		if (debugInfo.contains("System highlighted goal")) {
			explainSys = 1;
		}
		if (debugInfo.contains("Environment highlighted goal")) {
			explainEnv = 1;
		}
		
		return debugInfo;
	}
	
	
	
	public static class NOPPrintStream extends PrintStream
	{
	    public NOPPrintStream() { super((OutputStream)null); }

	    public void println(String s) { /* Do nothing */ }
	    // You may or may not have to override other methods
	}
	
	public static String justiceChecks(SMVModule env, SMVModule sys, int explainSys, int explainEnv, boolean falsifyEnv) throws GameException  {
		
		BDD all_init = sys.initial().and( env.initial());
		BDD counter_exmple;
		GROneGame g;
		
		String debugInfo = "";
		BDD prev;
		g = new GROneGame(env,sys, sys.justiceNum(), env.justiceNum());
			
		counter_exmple = g.envWinningStates().and(all_init);		 
		if (counter_exmple.isZero()) {	//no winning environment states		
			debugInfo += "Specification is realizable assuming instantaneous actions.\n";
		}	
			
		if (!(explainEnv == 1 || env.justiceNum()==1 && env.justiceAt(0).equals(Env.TRUE()))) {
			
			boolean flagRealPrev = false;
			
			for (int i = env.justiceNum(); i >=1; i--){		
				//checking for unrealizable environment justice 				
				if (env.justiceAt(i-1).isZero()) {
					debugInfo += "Environment highlighted goal(s) unsatisfiable: " + (i-1) + "\n";
					explainEnv = 1;
				}
				else if ((env.trans().and(Env.prime(env.justiceAt(i-1))).isZero()) | (env.trans().and(env.justiceAt(i-1))).isZero()) { 
					debugInfo += "Environment highlighted goal(s) inconsistent with transition relation: " + (i-1) + "\n";
					 explainEnv = 1;
				}
				
				 prev = counter_exmple;
				 g = new GROneGame(env,sys, sys.justiceNum(), i);
				 counter_exmple = g.envWinningStates().and(all_init);
				 
				
				 
				 if (explainEnv ==0 && counter_exmple.isZero() & falsifyEnv) {
					 if (g.calculate_strategy(3, all_init.and(g.sysWinningStates()), false)) { 
						 //checking for multi-step unsatisfiability between env transitions and goals
						 //since the first argument is 3, we allow all three kinds of system transitions
						 //so we are looking for livelock 	
						 debugInfo += "Environment highlighted goal(s) inconsistent with transition relation: " + (i-1) + "\n";
						 explainEnv = 1;
						 i = 0;
					 //} else if (counter_exmple.isZero() && !env.justiceAt(i-1).equals(Env.TRUE())) {// && (!prev.isZero())) {
					 } else if (i <= env.justiceNum() & !flagRealPrev) {
						 //if we get here, the env is unrealizable because of the current goal
						 debugInfo += "Environment highlighted goal(s) unrealizable: " + (i-1) + "\n";	
						 explainEnv = 1;
						 i = 0;					
					 }
					 flagRealPrev = true;	//flags that the previous set of livenesses was also realizable				 
				 } else if ((explainEnv ==0 && !counter_exmple.isZero() & flagRealPrev)) {					 
					 //if we get here, the env is unrealizable because of the previous goal (this is important mainly for the case where the last goal is unrealiable)
					 debugInfo += "Environment highlighted goal(s) unrealizable: " + (i) + "\n";	
					 explainEnv = 1;
					 i = 0;
				 }
						 
			}
		}
		
		//Commented out code replaces the environment justices with TRUE
		//Useful for system liveness unsatisfiability checks
		/*try {
			while (env.justiceNum() > 0) {
				env.popLastJustice();
			}
			env.addJustice(Env.TRUE());
		}	catch (Exception e){//Catch exception if any
			System.err.println("Error: " + e.getMessage());
		}*/
		
		if (!(explainSys == 1 || sys.justiceNum()==1 && sys.justiceAt(0).equals(Env.TRUE()))) {
			for (int i = 1; i <= sys.justiceNum(); i++){
				 if (sys.justiceAt(i-1).isZero()) {
					 debugInfo += "System highlighted goal(s) unsatisfiable: " + (i-1) + "\n";
					 explainSys = 1;
				 }
				 else if ((sys.trans().and(Env.prime(sys.justiceAt(i-1))).isZero()) | sys.trans().and(sys.justiceAt(i-1)).isZero()) {
					 debugInfo += "System highlighted goal(s) inconsistent with transition relation: " + (i-1) + "\n";	
				 	 explainSys = 1;
				 }
			 
				 g = new GROneGame(env,sys, i, env.justiceNum());
				 counter_exmple = g.envWinningStates().and(all_init);
				 if (explainSys ==0 && !counter_exmple.isZero()) {
					//checking for multi-step unsatisfiability between sys transitions and goals
					//since the second argument is true, we are looking for livelock 
					 if (g.calculate_counterstrategy(counter_exmple, true, false)) {
						 debugInfo += "System highlighted goal(s) inconsistent with transition relation: " + (i-1) + "\n";
						 explainSys = 1;
					 } else  {
					 //if we get here, the sys is unrealizable because of the current goal					 				
						 debugInfo += "System highlighted goal(s) unrealizable: " + (i-1) + "\n";
						 explainSys = 1;		
					 }
					 i = sys.justiceNum() + 1;
				 }
			}
		}
		return debugInfo;
	}
	
	public static String refine(SMVModule env, SMVModule sys, String[] args) {
		String result = "";
		try{
			result += "Guilty safety conjuncts are " + SpecSubsets.iteratedCoreSafetyIndices(args);
		}catch (Exception e){//Catch exception if any
			System.err.println("Error: " + e.getMessage());
		}
		return result;
	}
}