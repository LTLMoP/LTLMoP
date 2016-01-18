import edu.wis.jtlv.env.module.ModuleBDDField;
import net.sf.javabdd.BDD;
import net.sf.javabdd.BDDVarSet;
import net.sf.javabdd.BDDFactory;
import net.sf.javabdd.BDD.BDDIterator;
import edu.wis.jtlv.env.Env;
import edu.wis.jtlv.env.module.SMVModule;
import edu.wis.jtlv.env.spec.Spec;
import java.io.File;
import java.io.PrintStream;
import edu.wis.jtlv.lib.FixPoint;

/** 
 * GROneMain is the main front-end to the synthesis subsystem
 */

public class GROneMain {
    /*** Helper functions ***/

    private static void printUsage() {
        // Print command-line usage information
        System.err.println("Usage: java GROneMain <smv_file> <ltl_file> [--fastslow] [--safety] [--symbolic]");
        System.err.println("");
        System.err.println("Options: --fastslow (enable extra checks for safety of implicit intermediate states during synthesis)");
        System.err.println("         --safety   (in addition to strategy, export system transition relations)");
        System.err.println("         --symbolic (output BDDs instead of explicit-state automata)");
    }

    private static boolean synthesisSucceeded(GROneGame g) {
        // Return True iff the system can win from all initial states

        BDD all_init, counter_example;

        all_init = g.getSysPlayer().initial().and(g.getEnvPlayer().initial());
        counter_example = g.envWinningStates().and(all_init);

        return counter_example.isZero();
           
        // If you only care about the existence of a winning initial system state for every initial environment state
        // (vs. every initial system state being winning for every initial environment state):
        /*	
        BDD env_ini = g.getEnvPlayer().initial();
            BDDVarSet env_vars = g.getEnvPlayer().moduleUnprimeVars();
            for (BDDIterator it = env_ini.iterator(env_vars); it.hasNext();) {
                BDD eini = (BDD) it.next();
                BDD sys_response = eini.and(g.getSysPlayer().initial()).and(
                        g.sysWinningStates());
                if (sys_response.isZero()) {
                    return false;
                }
        }
        return true;
        */
    }

    /*** Main routine ***/

	public static void main(String[] args) throws Exception {
		// uncomment to use a C BDD package
		//System.setProperty("bdd", "buddy");

		GROneParser.just_initial = true; // what is this for?

        // Check that we have enough arguments
        if (args.length < 2) {
            printUsage();
            System.exit(1);
        }                

        /* Parse extra command-line switches */

        // defaults
        boolean opt_fs = false;
        boolean opt_gen_safety = false;
        boolean opt_symbolic = false;

        for (int i = 2; i < args.length; i++) {
            if (args[i].equals("--fastslow")) {
                opt_fs = true;
            } else if (args[i].equals("--safety")) {
                opt_gen_safety = true;
            } else if (args[i].equals("--symbolic")) {
                opt_symbolic = true;
            } else {
                System.err.println("Unknown option: " + args[i]);
                printUsage();
                System.exit(1);
            }
        }

        // Load SMV and LTL files
        Env.loadModule(args[0]);
        Spec[] spcs = Env.loadSpecFile(args[1]);
        
        // Figure out the base name for our output files by stripping the .smv filename extension
        String out_filename_base = args[1].replaceAll("\\.[^\\.]+$","");
        String out_filename;

		// Construct the environment module.
		SMVModule env = (SMVModule) Env.getModule("main.e");
		Spec[] env_conjuncts = GROneParser.parseConjuncts(spcs[0]);
		GROneParser.addReactiveBehavior(env, env_conjuncts);

		// Construct the system module.
		SMVModule sys = (SMVModule) Env.getModule("main.s");
		Spec[] sys_conjuncts = GROneParser.parseConjuncts(spcs[1]);
		GROneParser.addReactiveBehavior(sys, sys_conjuncts);

		// env.setFullPrintingMode(true);
		// System.out.println(env);
		// sys.setFullPrintingMode(true);
		// System.out.println(sys);

		// ** Play the game

		System.out.print("====> Checking synthesizability...\n");
	
        long tic, toc;
		GROneGame g = null;
		
		// If the option is enabled, first try fastslow	
		if (opt_fs) {
            tic = System.currentTimeMillis();		
			g = new GROneGame(env,sys,true);
		    toc = System.currentTimeMillis();
			System.out.println("Completed in: " + (toc-tic)/1000.0 + "sec");
			
            if (synthesisSucceeded(g)) {
                 System.out.println("Specification is synthesizable under fast/slow!");
            } else {
		     	 System.out.println("Unsynthesizable under fast/slow.  Falling back to normal...");
            }
        }

        // If fastslow failed or wasn't chosen, synthesize normally
        if (!opt_fs || !synthesisSucceeded(g)) {
            tic = System.currentTimeMillis();		
			g = new GROneGame(env,sys);
		    toc = System.currentTimeMillis();
			System.out.println("Completed in: " + (toc-tic)/1000.0 + "sec");
        }

		// ** Output a strategy, if one exists

        if (synthesisSucceeded(g)) {
            System.out.println("Specification is synthesizable!");
            System.out.println("====> Building an implementation...");

            out_filename = out_filename_base + (opt_symbolic ? ".add" : ".aut");

            tic = System.currentTimeMillis();		

            if (opt_symbolic) {
                DDDumper.writeStrategyBDDToFile(g, out_filename);
            } else {
                PrintStream orig_out = System.out;

                System.setOut(new PrintStream(new File(out_filename))); // redirect the output to a file

                BDD all_init = g.getSysPlayer().initial().and(g.getEnvPlayer().initial());
                boolean falsifyEnv = g.printWinningStrategy(all_init);
				if (falsifyEnv) {
					System.out.println("WARNING: Environment liveness falsified"); 
				}

                System.setOut(orig_out); // restore STDOUT
            }

            toc = System.currentTimeMillis();		

            System.out.println("Completed in: " + (toc-tic)/1000.0 + "sec");

            System.out.println("Strategy written to file: " + out_filename);

        } else {

            System.out.println("ERROR: Specification is unsynthesizable.");

			/* Give some basic feedback about losing initial states */
			BDD all_init = g.getSysPlayer().initial().and(g.getEnvPlayer().initial());
			BDD counter_example = g.envWinningStates().and(all_init);
        	BDDVarSet all_vars = sys.moduleUnprimeVars().union(env.moduleUnprimeVars());
		
        	System.out.println("The env player can win from " + (int)counter_example.satCount(all_vars) + " of " + (int)all_init.satCount(all_vars) +" initial state(s).");
            String counter_state = counter_example.satOne().toString();
            counter_state = counter_state.replaceAll("and", "&");
            counter_state = counter_state.replaceAll("[()]", "");
            counter_state = counter_state.replaceAll("main\\.([\\w.]+)=0", "!$1");
            counter_state = counter_state.replaceAll("main\\.([\\w.]+)=1", "$1");
			System.out.println("\tFor example: \t" + counter_state);
        }
		
		// ** Export safety automaton

		if (opt_gen_safety) {
			System.out.println("====> Exporting safety constraints automaton...");

			PrintStream orig_out = System.out;
			out_filename = out_filename_base + "_safety.aut";
            System.setOut(new PrintStream(new File(out_filename))); // redirect the output to a file

            tic = System.currentTimeMillis();		

			g.generate_safety_aut(g.getEnvPlayer().initial().and(
					g.getSysPlayer().initial()));

            toc = System.currentTimeMillis();		

			System.setOut(orig_out); // restore STDOUT

			System.out.println("Completed in: " + (toc-tic)/1000.0 + "sec");
		}

		

        if (!synthesisSucceeded(g)) {
			System.out.println("====> Performing unsynthesizability analysis...");
            tic = System.currentTimeMillis();		
            GROneDebug.analyze(env,sys);
            toc = System.currentTimeMillis();		
			System.out.println("Completed in: " + (toc-tic)/1000.0 + "sec");

			System.out.println("====> Computing counterstrategy...");

            // TODO: make the counterstrat aut have a distinct filename?
            // TODO: enable BDD export of counterstrat too
			out_filename = out_filename_base + ".aut";

            PrintStream orig_out = System.out;
            System.setOut(new PrintStream(new File(out_filename))); // writing the output to a file

            tic = System.currentTimeMillis();		

            BDD all_init, counter_example;

            all_init = g.getSysPlayer().initial().and(g.getEnvPlayer().initial());
            counter_example = g.envWinningStates().and(all_init);
            g.printLosingStrategy(counter_example);

            toc = System.currentTimeMillis();		

            System.setOut(orig_out); // restore STDOUT

			System.out.println("Completed in: " + (toc-tic)/1000.0 + "sec");
        }

        //return !synthesisSucceeded(g);
	}

}
