import net.sf.javabdd.BDD;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import edu.wis.jtlv.env.Env;
import edu.wis.jtlv.env.module.SMVModule;
import edu.wis.jtlv.env.spec.Spec;
import edu.wis.jtlv.env.spec.Operator;
import edu.wis.jtlv.env.spec.SpecBDD;
import edu.wis.jtlv.env.spec.SpecExp;

public class SpecSubsets {
    public enum ConjunctType {
        INITIAL, SAFETY, LIVENESS, OTHER;
    }

	public static void main(String[] args) throws Exception {
        // Check that we have enough arguments
        if (args.length < 2) {
            System.err.println("Usage: java SpecSubsets <smv_file> <ltl_file>");
            System.exit(1);
        }                

        // Load SMV and LTL files
        Env.loadModule(args[0]);
        Spec[] spcs = Env.loadSpecFile(args[1]);

        // Parse the specs
		Spec[] e_conjuncts = GROneParser.parseConjuncts(spcs[0]);
		Spec[] s_conjuncts = GROneParser.parseConjuncts(spcs[1]);
		
        // Load all of the environment spec
		SMVModule m_env = (SMVModule) Env.getModule("main.e");
		GROneParser.addReactiveBehavior(m_env, e_conjuncts);

		SMVModule m_sys = (SMVModule) Env.getModule("main.s");
        // Load all of the system initials and goals
        GROneParser.addReactiveBehavior(m_sys, getConjunctsByType(s_conjuncts, ConjunctType.INITIAL));
        GROneParser.addReactiveBehavior(m_sys, getConjunctsByType(s_conjuncts, ConjunctType.LIVENESS));
 
        m_sys.initial().printSet();
        
        // Incrementally load the system safeties
        for (Spec safety_conjunct : getConjunctsByType(s_conjuncts, ConjunctType.SAFETY)) {
            clearSpecPartFromModule(m_sys, ConjunctType.SAFETY); // Remove this line if you want to accumulate
            System.out.println("Adding " + safety_conjunct);
            GROneParser.addReactiveBehavior(m_sys, new Spec[]{safety_conjunct});
            m_sys.trans().printSet();
            System.out.println("=====================================");
        }
	}
	
    public static String iteratedCoreSafetyIndices (String[] args) throws Exception {
	return (iteratedCoreSafetyIndices (args, false));
    }
	
    public static String iteratedCoreSafetyIndices (String[] args, boolean verbose) throws Exception {
	
        // Check that we have enough arguments
        if (args.length < 2) {
            System.err.println("Usage: java SpecSubsets <smv_file> <ltl_file>");
            System.exit(1);
        }                

        // Load SMV and LTL files
        Spec[] spcs = Env.loadSpecFile(args[1]);

        // Parse the specs
		Spec[] e_conjuncts = GROneParser.parseConjuncts(spcs[0]);
		Spec[] s_conjuncts = GROneParser.parseConjuncts(spcs[1]);
		
        // Load all of the environment spec
		SMVModule m_env = (SMVModule) Env.getModule("main.e");
		GROneParser.addReactiveBehavior(m_env, e_conjuncts);

		SMVModule m_sys = (SMVModule) Env.getModule("main.s");
        // Load all of the system initials and goals
        GROneParser.addReactiveBehavior(m_sys, getConjunctsByType(s_conjuncts, ConjunctType.INITIAL));
        GROneParser.addReactiveBehavior(m_sys, getConjunctsByType(s_conjuncts, ConjunctType.LIVENESS));
 
	Spec[] all_safeties = getConjunctsByType(s_conjuncts, ConjunctType.SAFETY);
	GROneParser.addReactiveBehavior(m_sys, all_safeties);
 
 
        
	BDD all_init = m_sys.initial().and(m_env.initial());
	if (verbose) {
	    System.out.println("Initial Conditions " + all_init);
	    System.out.println("Full safety specification " + m_sys.trans());
	}
	
	
	BDD counter_exmple;
	GROneGame g;
		
        
	int i = 0;
	int j = 0;
							       
	
	List<Integer> list = new LinkedList<Integer>();
	for (int k = 0; k <all_safeties.length; k++) {
	    list.add(k);
	}
    
	//populate list of core candidates with all conjuncts
	ArrayList<Integer> candidates = new ArrayList<Integer>(list);
	
	
        // Remove the system safeties one at a time
        for (Spec safety_conjunct : all_safeties) {
	    if (verbose) System.out.println("Removing conjunct " + i + " = " + safety_conjunct);
	    clearSpecPartFromModule(m_sys, ConjunctType.SAFETY); 
	    j = 0;
	    for (Spec safety_conjunct2 : all_safeties) {
		//add only those conjuncts that are being considered, excluding the current conjunct
	    	if (i != j && candidates.contains(new Integer(j))) {
		    GROneParser.addReactiveBehavior(m_sys, new Spec[]{safety_conjunct2});
		}
		j++;
	    }
	    
	    //check synthesizability
	    g = new GROneGame(m_env,m_sys);
	    counter_exmple = g.envWinningStates().and(all_init);
	    //if removing this conjunct has made the specification synthesizable, add it back in
	    if (counter_exmple.isZero()) {
		if (verbose) System.out.println("Added it back in" + safety_conjunct);
	    } else {
		//this is no longer a candidate guilty conjunct
		candidates.remove(new Integer(i));
	    }
	    if (verbose) System.out.println("=====================================");
            i++;
        }
	
	int[] ret = new int[candidates.size()];
	for(int n = 0;n < ret.length;n++) {
	  ret[n] = ((Integer)candidates.get(n)).intValue();
	}
	return Arrays.toString(ret);
    }

    public static Spec[] getConjunctsByType(Spec[] sp, ConjunctType t) throws Exception {
        ArrayList<Spec> conjuncts = new ArrayList<Spec>();
        
        for (Spec conjunct : sp) {
            if (getConjunctType(conjunct) == t) {
                conjuncts.add(conjunct);
            }
        }
          
        return conjuncts.toArray(new Spec[0]);
    }

    public static String getOutermostTemporalOperators(Spec sp) throws Exception {
        // Stop recursion when we reach a pure BDD
        if (sp instanceof SpecBDD) {
            return "";
        }

        // This should never happen
        if (!(sp instanceof SpecExp)) {
            throw new Exception("Encountered subconjunct that is neither SpecBDD or SpecExp");
        }
    
        // Cast to SpecExp
        SpecExp spe = (SpecExp) sp;
        
        // Add the temporal operator we see and recurse
        if (spe.getOperator() == Operator.GLOBALLY) {
            return "G" + getOutermostTemporalOperators(spe.getChildren()[0]);
        } else if (spe.getOperator() == Operator.FINALLY) {
            return "F" + getOutermostTemporalOperators(spe.getChildren()[0]);
        } else {
            throw new Exception("Encountered SpecExp that is neither GLOBALLY or FINALLY");
        }
    }

    public static ConjunctType getConjunctType(Spec sp) throws Exception {
        String operators = getOutermostTemporalOperators(sp);

        if (operators.equals("")) {
            return ConjunctType.INITIAL;
        } else if (operators.equals("G")) {
            return ConjunctType.SAFETY;
        } else if (operators.equals("GF")) {
            return ConjunctType.LIVENESS;
        } else {
            return ConjunctType.OTHER;
        }
    }

    private static void clearSpecPartFromModule(SMVModule m, ConjunctType t) throws Exception {
        // For some unknown reason, removeAllIniRestrictions() and removeAllTransRestrictions() don't seem to work
        if (t == ConjunctType.INITIAL) {
            m.setInitial(Env.TRUE());
        } else if (t == ConjunctType.SAFETY) {
            m.disjunctTrans(Env.TRUE());
        } else if (t == ConjunctType.LIVENESS) {
            // Gotta clear these out one by one, apparently
            while (m.justiceNum() > 0) {
                m.popLastJustice();
            }
        }
    }
}
