import java.util.Iterator;
import java.util.Stack;
import java.util.Vector;

import net.sf.javabdd.BDD;
import net.sf.javabdd.BDDVarSet;
import net.sf.javabdd.BDD.BDDIterator;
import edu.wis.jtlv.env.Env;
import edu.wis.jtlv.env.module.ModuleWithWeakFairness;
import edu.wis.jtlv.env.module.Module;
import edu.wis.jtlv.lib.FixPoint;
import edu.wis.jtlv.old_lib.games.GameException;
import edu.wis.jtlv.env.module.ModuleBDDField;

/** 
 * <p>
 * To execute, create an object with two Modules, one for the system and the
 * other for the environment, and then just extract the strategy/counterstrategy 
 * via the printWinningStrategy() and printLosingStrategy() methods.
 * </p>
 * 
 * @author Yaniv Sa'ar, Vasumathi Raman, Cameron Finucane)
 * 
 */
public class GROneGame {
	private ModuleWithWeakFairness env;
	private ModuleWithWeakFairness sys;
	int sysJustNum, envJustNum;
	private BDD player1_winning;
	private BDD player2_winning;
	
	BDD slowSame, fastSame, envSame;

	BDDVarSet sys_slow_prime;

	BDDVarSet sys_fast_prime;	
	
	boolean fastslow;

	
	public GROneGame(ModuleWithWeakFairness env, ModuleWithWeakFairness sys)
			throws GameException {
		this(env, sys, false); // default fastslow == false
	}

    public GROneGame(ModuleWithWeakFairness env, ModuleWithWeakFairness sys, boolean fastslow)
			throws GameException {
		this(env, sys, sys.justiceNum(), env.justiceNum(), fastslow);
    }
	
	public GROneGame(ModuleWithWeakFairness env, ModuleWithWeakFairness sys, int sysJustNum, int envJustNum)
			throws GameException {
		this(env, sys, sysJustNum, envJustNum, false); // default fastslow == false
    }

	public GROneGame(ModuleWithWeakFairness env, ModuleWithWeakFairness sys, int sysJustNum, int envJustNum, boolean fastslow)
			throws GameException {
		if ((env == null) || (sys == null)) {
			throw new GameException(
					"cannot instanciate a GR[1] Game with an empty player.");
		}
		
		//Define system and environment modules, and how many liveness conditions are to be considered. 
		//The first sysJustNum system livenesses and the first envJustNum environment livenesses will be used
		this.env = env;
		this.sys = sys;
		this.sysJustNum = sysJustNum;
		this.envJustNum = envJustNum;
		this.fastslow = fastslow;
		
		// for now I'm giving max_y 50, at the end I'll cut it (and during, I'll
		// extend if needed. (using vectors only makes things more complicate
		// since we cannot instantiate vectors with new vectors)
		x_mem = new BDD[sysJustNum][envJustNum][50];
		y_mem = new BDD[sysJustNum][50];
		z_mem = new BDD[sysJustNum];
		x2_mem = new BDD[sysJustNum][envJustNum][50][50];
		y2_mem = new BDD[sysJustNum][50];
		z2_mem = new BDD[50];	
			
		
        if (fastslow) {
            this.player2_winning = this.calculate_win_FS();	//(system winning states)
            // p2_winning in GRGAmes are !p1_winning
            this.player1_winning = this.player2_winning.not(); //(environment winning states)
        } else {
            this.player2_winning = this.calculate_win();	//(system winning states)
            this.player1_winning = this.calculate_loss();   //(environment winning states)
        }

	}
	
	


	public BDD[][][] x_mem;
	public BDD[][][][] x2_mem;
	public BDD[][] y_mem, y2_mem;
	public BDD[] z_mem, z2_mem;

	/**
	 * <p>
	 * Calculating Player-2 winning states.
	 * </p>
	 * 
	 * @return The Player-2 winning states for this game.
	 */
	private BDD calculate_win() {
		BDD x, y, z;
		FixPoint<BDD> iterZ, iterY, iterX;
		int cy = 0;

		z = Env.TRUE();
		for (iterZ = new FixPoint<BDD>(); iterZ.advance(z);) {
			//for (int j = 0; j < sys.justiceNum(); j++) {
			for (int j = 0; j < sysJustNum; j++) {
				cy = 0;
				y = Env.FALSE();
				for (iterY = new FixPoint<BDD>(); iterY.advance(y);) {
                    BDD y2 = sys.justiceAt(j).and(env.yieldStates(sys, z)).or(y);
                    for (FixPoint<BDD> iterY2 = new FixPoint<BDD>(); iterY2.advance(y2);) {
                        y_mem[j][cy] = y2.id();
                        for (int i = 0; i < envJustNum; i++) {
                            x_mem[j][i][cy] = y2.id();
                        }
                        cy++;
                        if (cy % 50 == 0) {
                            x_mem = extend_size(x_mem, cy);
                            y_mem = extend_size(y_mem, cy);
                        }
                        y2 = y2.or(env.yieldStates(sys, y2));
                    }
                    BDD start = y2.id(); 

					y = Env.FALSE();
					for (int i = 0; i < envJustNum; i++) {
						BDD negp = env.justiceAt(i).not();
						x = z.id();
						for (iterX = new FixPoint<BDD>(); iterX.advance(x);) {
							x = negp.and(env.yieldStates(sys, x)).or(start);
						}
						x_mem[j][i][cy] = x.id();
						//System.out.println("X ["+ j + ", " + i + ", " + cy + "] = " + x_mem[j][i][cy]);							
						y = y.id().or(x);
					}
					y_mem[j][cy] = y.id();
					//System.out.println("Y ["+ j + "] = " + y_mem[j][cy]);													
					cy++;
					if (cy % 50 == 0) {
						x_mem = extend_size(x_mem, cy);
						y_mem = extend_size(y_mem, cy);
					}
				}
				z = y.id();
				z_mem[j] = z.id();
				//System.out.println("Z ["+ j + "] = " + z_mem[j]);							
						
			}
		}
		x_mem = extend_size(x_mem, 0);
		y_mem = extend_size(y_mem, 0);	
		return z.id();
	}
	
	/**
	 * <p>
	 * Calculating Player-2 losing states.
	 * </p>
	 * 
	 * @return The Player-2 losing states for this game.
	 */
	
	private BDD calculate_win_FS() {	
		
		BDD x, y, z;
		FixPoint<BDD> iterZ, iterY, iterX;
		int cy = 0;

		z = Env.TRUE();
		for (iterZ = new FixPoint<BDD>(); iterZ.advance(z);) {
			for (int j = 0; j < sysJustNum; j++) {
				cy = 0;
				y = Env.FALSE();
				for (iterY = new FixPoint<BDD>(); iterY.advance(y);) {
					BDD start = sys.justiceAt(j).and(yieldStatesFS(env,sys, z))
							.or(yieldStatesFS(env,sys, y));
					y = Env.FALSE();
					for (int i = 0; i < envJustNum; i++) {
						BDD negp = env.justiceAt(i).not();
						x = z.id();
						for (iterX = new FixPoint<BDD>(); iterX.advance(x);) {
							x = negp.and(yieldStatesFS(env,sys, x)).or(start);
						}
						x_mem[j][i][cy] = x.id();
						y = y.id().or(x);
					}
					y_mem[j][cy] = y.id();
					cy++;
					if (cy % 50 == 0) {
						x_mem = extend_size(x_mem, cy);
						y_mem = extend_size(y_mem, cy);
					}
				}
				z = y.id();
				z_mem[j] = z.id();						
						
			}
		}
		x_mem = extend_size(x_mem, 0);
		y_mem = extend_size(y_mem, 0);				
		return z.id();
	}
	
    // COX	

	/*public BDD yieldStates(Module env, Module sys, BDD to) {

		BDDVarSet env_prime = env.modulePrimeVars();
		BDDVarSet sys_prime = sys.modulePrimeVars();
		BDD exy1 = (Env.prime(to).and(sys.trans())).exist(sys_slow_prime);		
		BDD exy3 = Env.unprime(Env.prime(to).and(sys.trans()).and(fastSame).exist(sys_prime));		
		BDD exy2 = (sys.trans().and(slowSame.and(exy3))).exist(sys_slow_prime);
		return env.trans().imp((exy1.and(exy2)).exist(sys_fast_prime)).forAll(env_prime);
	}*/
	
	public BDD controlStates(Module this1, Module responder, BDD to) {
		BDDVarSet responder_prime = responder.modulePrimeVars();
		BDDVarSet this_prime = this1.modulePrimeVars();
		BDD exy = responder.trans().imp(Env.prime(to)).forAll(responder_prime);
		return this1.trans().and(exy).exist(this_prime);
	}

	public BDD controlStatesX(Module this1, Module responder, BDD to) {
		BDDVarSet responder_prime = responder.modulePrimeVars();
		BDDVarSet this_prime = this1.modulePrimeVars();
		BDD exy = responder.trans().imp(Env.prime(to)).forAll(responder_prime);
		return (this1.trans()).and(exy);
	}


	

    //COX_FS

	public BDD yieldStatesFS(Module env, Module sys, BDD to) {
		
		//Construct the BDDs required for fastSlow
		BDDVarSet sys_slow_prime = Env.getEmptySet();
		BDDVarSet sys_fast_prime = Env.getEmptySet();
		BDDVarSet sys_slow_unprime = Env.getEmptySet();
		BDDVarSet sys_fast_unprime = Env.getEmptySet();
		BDD slowSame = Env.TRUE();
		BDD fastSame = Env.TRUE();
		BDD envSame = Env.TRUE();


		ModuleBDDField [] allFields = sys.getAllFields();				
		ModuleBDDField thisField, thisPrime;
		String fieldName;

		for (int i = 0; i < allFields.length; i++) {
			thisField = allFields[i];
			thisPrime = thisField.prime();	
			fieldName = thisPrime.support().toString();
			if (fieldName.startsWith("<bit")) {
				slowSame = slowSame.id().and(thisField.getDomain().buildEquals(thisField.getOtherDomain()));
				sys_slow_prime = sys_slow_prime.union(thisPrime.support());
				sys_slow_unprime = sys_slow_unprime.union(thisField.support());				
			} else {
				fastSame = fastSame.id().and(thisField.getDomain().buildEquals(thisField.getOtherDomain()));
				sys_fast_prime = sys_fast_prime.union(thisPrime.support());
				sys_fast_unprime = sys_fast_unprime.union(thisField.support());
			}		
		}

		allFields = env.getAllFields();			
		for (int i = 0; i < allFields.length; i++) {
			thisField = allFields[i];
			thisPrime = thisField.prime();
			fieldName = thisPrime.support().toString();			
			envSame = envSame.id().and(thisField.getDomain().buildEquals(thisField.getOtherDomain()));			
		}	

		BDDVarSet env_prime = env.modulePrimeVars();
		BDDVarSet sys_prime = sys.modulePrimeVars();
		
		BDD safeStates = sys.trans().exist(env.modulePrimeVars().union(sys.modulePrimeVars()));
		BDD safeNext = Env.unprime(sys.trans().exist(env.moduleUnprimeVars().union(sys.moduleUnprimeVars())));
		
		//Read the new cox starting at the return statement***
		
		//And there is a transition to the "to" set, i.e. a slow move as well as the (pre-determined) fast move
		BDD exy1 = Env.prime(to).and(sys.trans()).exist(sys_slow_prime);	
		
		
		//Such that the slowSame move is safe...
		BDD exy6 = (slowSame.imp(Env.prime(safeStates.and(safeNext)))).forAll(sys_slow_prime);	
		
		
		//If both types are changing, check the intermediate state (i.e. the slowSame state):
		
		//There exists a "fast" move...
		BDD fs1 = (exy1.and(exy6)).exist(sys_fast_prime);
		
		
		//If only one type of controller changes, use old cox
		BDD fs2 = Env.prime(to).and(sys.trans()).and(slowSame.or(fastSame)).exist(sys_prime);

		
		///***For all environment moves, there is either a move that changes only slow or only fast, 
		//or one that changes both actions but has a safe intermediate state
		return env.trans().imp(fs1.or(fs2)).forAll(env_prime);
	}
	
	
	private BDD calculate_loss() {
		BDD x, y, z;
		FixPoint<BDD> iterZ, iterY, iterX;
		int c = 0;
		int a = 0;
		z = Env.FALSE();
		x = Env.FALSE();
				
		for (iterZ = new FixPoint<BDD>(); iterZ.advance(z);) {
			for (int j = 0; j < sysJustNum; j++) {							
				y = Env.TRUE();			
				for (iterY = new FixPoint<BDD>(); iterY.advance(y);) {
					BDD start = ((sys.justiceAt(j).not()).or(controlStates(env, sys, z)))
								.and(controlStates(env, sys, y));
					for (int i = 0; i < envJustNum; i++) {
						x = Env.FALSE();
						c=0;
						for (iterX = new FixPoint<BDD>(); iterX.advance(x);) {
							x = x.id().or((env.justiceAt(i).or(((controlStates(env, sys, x))))).and(start));
							x2_mem[j][i][a][c] = x.id();
							//System.out.println("X ["+ j + ", " + i + ", " + a + ", " + c + "] = " + x2_mem[j][i][a][c]);
							c++;
						}
						
						y = y.id().and(x);
					}				
					if (c % 50 == 0) {
						x2_mem = extend_size(x2_mem, c);						
					}					
				}
				y2_mem[j][a] = y.id();	
				//System.out.println("Y ["+ j + ", " + c + "] = " + y2_mem[j][a]);

				z = z.id().or(y);				
			}
			
			
			z2_mem[a] = z.id();
			//System.out.println("Z ["+ a + "] = " + z2_mem[a]);
			a++;
			if (a % 50 == 0) {
				z2_mem = extend_size(z2_mem, a);
				y2_mem = extend_size(y2_mem, a);
			}
			
		}
		
		x2_mem = extend_size(x2_mem, 0);
		y2_mem = extend_size(y2_mem, 0);
		z2_mem = extend_size(z2_mem, 0);
		return z.id();
	}

	private BDD[][][][] extend_size(BDD[][][][] in, int extended_size) {
		BDD[][][][] res;
		if (extended_size > 0) {
			res = new BDD[in.length][in[0].length][in[0][0].length + extended_size][in[0][0][0].length
					+ extended_size];
			for (int i = 0; i < in.length; i++) {
				for (int j = 0; j < in[i].length; j++) {
					for (int k = 0; k < in[i][j].length; k++) {
						for (int m = 0; m < in[i][j][k].length; m++) {
							res[i][j][k][m] = in[i][j][k][m];
						}
					}
				}
			}
		} else {
			res = new BDD[in.length][in[0].length][in[0][0].length][];
			for (int i = 0; i < in.length; i++) {
				for (int j = 0; j < in[i].length; j++) {
					for (int k = 0; k < in[i][j].length; k++) {						
						int real_size = 0;
						for (int m = 0; m < in[i][j][k].length; m++) {
							if (in[i][j][k][m] != null)
								real_size++;
						}
						res[i][j][k] = new BDD[real_size];
						int new_add = 0;
						for (int m = 0; m < in[i][j][k].length; m++) {
							if (in[i][j][k][m] != null) {
								res[i][j][k][new_add] = in[i][j][k][m];
								new_add++;
							}
						}
					}
				}
			}
		}
		return res;
	}	
	// extended_size<=0 will tight the arrays to be the exact sizes.
	
	private BDD[][][] extend_size(BDD[][][] in, int extended_size) {
		BDD[][][] res;
		if (extended_size > 0) {
			res = new BDD[in.length][in[0].length][in[0][0].length
					+ extended_size];
			for (int i = 0; i < in.length; i++) {
				for (int j = 0; j < in[i].length; j++) {
					for (int k = 0; k < in[i][j].length; k++) {
						res[i][j][k] = in[i][j][k];
					}
				}
			}
		} else {
			res = new BDD[in.length][in[0].length][];
			for (int i = 0; i < in.length; i++) {
				for (int j = 0; j < in[i].length; j++) {
					int real_size = 0;
					for (int k = 0; k < in[i][j].length; k++) {
						if (in[i][j][k] != null)
							real_size++;
					}
					res[i][j] = new BDD[real_size];
					int new_add = 0;
					for (int k = 0; k < in[i][j].length; k++) {
						if (in[i][j][k] != null) {
							res[i][j][new_add] = in[i][j][k];
							new_add++;
						}
					}
				}
			}
		}
		return res;
	}

	private BDD[][] extend_size(BDD[][] in, int extended_size) {
		BDD[][] res;
		if (extended_size > 0) {
			res = new BDD[in.length][in[0].length + extended_size];
			for (int i = 0; i < in.length; i++) {
				for (int j = 0; j < in[i].length; j++) {
					res[i][j] = in[i][j];
				}
			}
		} else {
			res = new BDD[in.length][];
			for (int i = 0; i < in.length; i++) {
				int real_size = 0;
				for (int j = 0; j < in[i].length; j++) {
					if (in[i][j] != null)
						real_size++;
				}
				res[i] = new BDD[real_size];
				int new_add = 0;
				for (int j = 0; j < in[i].length; j++) {
					if (in[i][j] != null) {
						res[i][new_add] = in[i][j];
						new_add++;
					}
				}
			}
		}
		return res;
	}
	
	private BDD[] extend_size(BDD[] in, int extended_size) {
		BDD[] res;		
		if (extended_size > 0) {
			res = new BDD[in.length + extended_size];
			for (int i = 0; i < in.length; i++) {
				
					res[i] = in[i];
				
			}
		} else {
				int real_size = 0;
				for (int j = 0; j < in.length; j++) {
					if (in[j] != null)
						real_size++;
				}
				res = new BDD[real_size];
				for (int j = 0; j < real_size; j++) {
					
						res[j] = in[j];
				
				}
		}			
		return res;
	}

	/**
	 * <p>
	 * Extracting an arbitrary implementation from the set of possible
	 * strategies. 
	 * The second argument changes the priority of searching for different types of moves in the game
	 * </p>
	 */
	public boolean printWinningStrategy(BDD ini) {
		return calculate_strategy(3, ini);
		// return calculate_strategy(3);
		// return calculate_strategy(7);
		// return calculate_strategy(11);
		// return calculate_strategy(15);
		// return calculate_strategy(19);
		// return calculate_strategy(23);
	}
	
	/**
	 * <p>
	 * Extracting an arbitrary counterstrategy from the set of possible counterstrategies. 
	 * </p>
	 */
	
	public void printLosingStrategy(BDD ini) {
		calculate_counterstrategy(ini);
		// return calculate_strategy(3);
		// return calculate_strategy(7);
		// return calculate_strategy(11);
		// return calculate_strategy(15);
		// return calculate_strategy(19);
		
	}

	
	
	/**
	 * <p>
	 * Extracting an implementation from the set of possible strategies with the
	 * given priority to the next step, following the approach outlined in	
	 * </p>
	 * <p>
	 * Nir Piterman, Amir Pnueli, and Yaniv Sa'ar. Synthesis of Reactive(1) Designs.
	 * In VMCAI 2006, pp. 364-380.
	 * </p>
	 * <p>
	 * Possible priorities are:<br>
	 * 3 - Z Y X.<br>
	 * 7 - Z X Y.<br>
	 * 11 - Y Z X.<br>
	 * 15 - Y X Z.<br>
	 * 19 - X Z Y.<br>
	 * 23 - X Y Z.<br>
	 * </p>
	 * 
	 * @param kind
	 *            The priority kind.
	 * @param det
	 *            true if a deterministic strategy is desired, otherwise false
	 */
	
		
	public boolean calculate_strategy(int kind, BDD ini, boolean det) {
		
		BDDVarSet sys_slow_prime = Env.getEmptySet();
		BDDVarSet sys_fast_prime = Env.getEmptySet();
		BDDVarSet sys_slow_unprime = Env.getEmptySet();
		BDDVarSet sys_fast_unprime = Env.getEmptySet();
		BDD slowSame = Env.TRUE();
		BDD fastSame = Env.TRUE();
		BDD envSame = Env.TRUE();


		ModuleBDDField [] allFields = sys.getAllFields();				
		ModuleBDDField thisField, thisPrime;
		String fieldName;

		for (int i = 0; i < allFields.length; i++) {
			thisField = allFields[i];
			thisPrime = thisField.prime();	
			fieldName = thisPrime.support().toString();
			if (fieldName.startsWith("<bit")) {
				slowSame = slowSame.id().and(thisField.getDomain().buildEquals(thisField.getOtherDomain()));
				sys_slow_prime = sys_slow_prime.union(thisPrime.support());
				sys_slow_unprime = sys_slow_unprime.union(thisField.support());				
			} else {
				fastSame = fastSame.id().and(thisField.getDomain().buildEquals(thisField.getOtherDomain()));
				sys_fast_prime = sys_fast_prime.union(thisPrime.support());
				sys_fast_unprime = sys_fast_unprime.union(thisField.support());
			}		
		}
		
		BDDVarSet env_prime = env.modulePrimeVars();
		BDDVarSet sys_prime = sys.modulePrimeVars();
		
		BDD safeStates = sys.trans().exist(env.modulePrimeVars().union(sys.modulePrimeVars()));
		BDD safeNext = Env.unprime(sys.trans().exist(env.moduleUnprimeVars().union(sys.moduleUnprimeVars())));
		
		
		int strategy_kind = kind;
		Stack<BDD> st_stack = new Stack<BDD>();
		Stack<Integer> j_stack = new Stack<Integer>();
		Stack<RawState> aut = new Stack<RawState>();
		boolean result;
		if (ini.isZero()) result = false; else result = true;
		
        // Create a varset of all non-location propositions
        // (used later to prevent inefficient wobbling when
		// multiple goals are satisfied in the same location)
        //
        // TODO: There is probably a cleaner way to do this
        //ModuleBDDField [] allFields = sys.getAllFields();        
        BDDVarSet nonRegionProps = Env.getEmptySet();
        for (int i = 0; i < allFields.length; i++) {      
          thisField = allFields[i];
          fieldName = thisField.support().toString();
          if (!fieldName.startsWith("<bit"))
            nonRegionProps = nonRegionProps.union(thisField.support());
        }
        

        BDDIterator ini_iterator = ini.iterator(env.moduleUnprimeVars().union(sys.moduleUnprimeVars()));
        
        boolean falsifyEnv = true;

        
		while (ini_iterator.hasNext()) {
            BDD this_ini = (BDD) ini_iterator.next();

            RawState test_st = new RawState(aut.size(), this_ini, 0);

            int idx = -1;
            for (RawState cmp_st : aut) {
                if (cmp_st.equals(test_st, false)) { // search ignoring rank
                    idx = aut.indexOf(cmp_st);
                    break;
                }
            }

            if (idx != -1) {
                // This initial state is already in the automaton
                continue;
            }

            // Otherwise, we need to attach this initial state to the automaton

            st_stack.push(this_ini);
            j_stack.push(new Integer(0)); // TODO: is there a better default j?
            //this_ini.printSet();
            
            //keep track of whether the winning strategy ever falsifies the environment
            
            // iterating over the stacks.
            while (!st_stack.isEmpty()) {
    			// making a new entry.
                BDD p_st = st_stack.pop();
                int p_j = j_stack.pop().intValue();

                /* 
                * Create a new automaton state for our current state 
                * (or use a matching one if it already exists) 
            	* p_st is the current state value,
				* and p_j is the system goal currently being pursued.
				* cf. RawState class definition below.
				*/
                RawState new_state = new RawState(aut.size(), p_st, p_j);
                int nidx = aut.indexOf(new_state);
                if (nidx == -1) {
                    aut.push(new_state);
                } else {
                    new_state = aut.elementAt(nidx);
                }

                /* Find Y index of current state */
                // find minimal cy and an i
                int p_cy = -1;
                for (int i = 0; i < y_mem[p_j].length; i++) {
                    if (!p_st.and(y_mem[p_j][i]).isZero()) {
                        p_cy = i;
                        break;
                    }
                }
                assert p_cy >= 0 : "Couldn't find p_cy";
                /* Find  X index of current state */
                int p_i = -1;
                for (int i = 0; i < envJustNum; i++) {
			         if (!p_st.and(x_mem[p_j][i][p_cy]).isZero()) {
                        p_i = i;
                        break;
                    }
                }
                assert p_i >= 0 : "Couldn't find p_i";

                // computing the set of env possible successors.
                Vector<BDD> succs = new Vector<BDD>();
                BDD all_succs = Env.FALSE();
                if (det) {
                	all_succs = env.succ(p_st);
                } else {
                	all_succs = Env.TRUE();
                }
/*              if (all_succs.isZero()) {
					//System.out.println("No successor was found"+p_st);
					return true;
					// This is just wrong, and was causing the empty automaton problem seen by Bingxin					
				}
*/
                for (BDDIterator all_states = all_succs.iterator(env
                        .moduleUnprimeVars()); all_states.hasNext();) {
                    BDD sin = (BDD) all_states.next();
                    succs.add(sin);
                }

                BDD candidate = Env.FALSE();
                // For each env successor, find a strategy successor
                for (Iterator<BDD> iter_succ = succs.iterator(); iter_succ
                        .hasNext();) {
                    BDD primed_cur_succ = Env.prime(iter_succ.next());
                    BDD next_op;
					if (this.fastslow) {
						//for fastSlow, need to make sure we only choose successors that have a safe intermediate state					
	                    next_op = Env.unprime(sys.trans().and(p_st).and(primed_cur_succ)
	                    					.and((slowSame.or(fastSame))
	                    							.or(slowSame.and(Env.prime(safeStates.and(safeNext))).exist(sys_slow_prime)))
	                                    .exist(
	                                            env.moduleUnprimeVars().union(
	                                                    sys.moduleUnprimeVars())));
					} else {
						next_op = Env.unprime(sys.trans().and(p_st).and(primed_cur_succ)
                            .exist(
                                    env.moduleUnprimeVars().union(
                                            sys.moduleUnprimeVars())));
					}
					
                    candidate = Env.FALSE();
                    int jcand = p_j;

                    int local_kind = strategy_kind;
                    while (candidate.isZero() & (local_kind >= 0)) {
                        // a - first successor option in the strategy.
                        // (rho_1 in Piterman; satisfy current goal and move to next goal)
                        if ((local_kind == 3) | (local_kind == 7)
                                | (local_kind == 10) | (local_kind == 13)
                                | (local_kind == 18) | (local_kind == 21)) {
                            if (!p_st.and(sys.justiceAt(p_j)).isZero()) {
								int next_p_j = (p_j + 1) % sysJustNum;   
								
								// Cycle through goals that are trivially satisfied by staying in the exact same state.
                                // (This is essentially stutter-state removal)
								while (!p_st.and(sys.justiceAt(next_p_j)).isZero() && next_p_j != p_j)
								{
									next_p_j = (next_p_j + 1) % sysJustNum;		
								}

                                // Find the lowest-rank transitionable state in the direction of the next unsatisfied goal
                                // (or just stay in the same y-set if we've looped all the way around)
                                int look_r = 0;
                                while ((next_op.and(y_mem[next_p_j][look_r]).isZero())) {
                                    look_r++;
                                }
                                    
                                BDD opt = next_op.and(y_mem[next_p_j][look_r]);
                                if (!opt.isZero()) {
                                    candidate = opt;
                                    falsifyEnv = false;
                                    //System.out.println("1");
                                    jcand = next_p_j;
                                }
                            }
                        }                       
                        // b - second successor option in the strategy.
                        // (rho_2 in Piterman; move closer to current goal)
                        if ((local_kind == 2) | (local_kind == 5)
                                | (local_kind == 11) | (local_kind == 15)
                                | (local_kind == 17) | (local_kind == 22)) {
                            if (p_cy > 0) {
                                int look_r = 0;
                                // look for the farest r.
                                while ((next_op.and(y_mem[p_j][look_r]).isZero())
                                        & (look_r < p_cy)) {
                                    look_r++;
                                }
                                
                                BDD opt = next_op.and(y_mem[p_j][look_r]);
                                if ((look_r != p_cy) && (!opt.isZero())) {
                                    candidate = opt;  
                                    //System.out.println("2");
                                }
                            }
                        }

                        // c - third successor option in the strategy.
                        // (rho_3 in Piterman; falsify environment :()
                        if ((local_kind == 1) | (local_kind == 6)
                                | (local_kind == 9) | (local_kind == 14)
                                | (local_kind == 19) | (local_kind == 23)) {
                            if (!p_st.and(env.justiceAt(p_i).not()).isZero()) {
                                BDD opt = next_op.and(x_mem[p_j][p_i][p_cy]);
                                if (!opt.isZero()) {
                                    candidate = opt;
                                    //System.out.println("3");
                                }
                            }
                        }

                        // no successor was found yet.
                        //assert ((local_kind != 0) & (local_kind != 4)
                        if (!((local_kind != 0) & (local_kind != 4)
                                & (local_kind != 8) & (local_kind != 12)
                                & (local_kind != 16) & (local_kind != 20))) {
                        	//System.out.println("No successor was found");
                        	if (strategy_kind == 3 && !det) return false;
							//This next line is critical to finding transitions at the lowest "level sets"
							else candidate = (next_op).and(y_mem[p_j][p_cy]);	
							assert !(det && candidate.isZero()) : "No successor was found";
														
                        }

                        local_kind--;
                    }


                    // If possible, trim down the candidates to prioritize movement-less transitions
                    BDD current_region = p_st.exist(env.moduleUnprimeVars()).exist(nonRegionProps);
                    if (!candidate.and(current_region).isZero()) {
                        candidate = candidate.and(current_region); 
                    }
                      	
                    for (BDDIterator candIter = candidate.iterator(env.moduleUnprimeVars().union(
                            sys.moduleUnprimeVars())); candIter.hasNext();) {
                        BDD one_cand = (BDD) candIter.next();
                                   
                        RawState gsucc = new RawState(aut.size(), one_cand, jcand);
                        idx = aut.indexOf(gsucc); // the equals doesn't consider
                                              // the id number.
                        if (idx == -1) {
	                        st_stack.push(one_cand);
	                        j_stack.push(jcand);
	                        aut.add(gsucc);
	                        idx = aut.indexOf(gsucc);
                        }
	                    new_state.add_succ(aut.elementAt(idx));
	                    if (det) break; //if we only need one successor, stop here
                    }
                    //when detecting environment unsatisfiability, all system actions should be valid from primed_cur_succ
                    //result = result & (candidate.equals(sys.trans().and(env.trans())));
                    result = result & (candidate.equals(primed_cur_succ));
                }
            }
		}

        /* Remove stuttering */
        // TODO: Make this more efficient (and less ugly) if possible
        /*
		int num_removed = 0;
      	   
        for (RawState state1 : aut) {
            int j1 = state1.get_rank();
            for (RawState state2 : state1.get_succ()) {
                int j2 = state2.get_rank();
                if ((j2 == (j1 + 1) % sys.justiceNum()) &&
                     state1.equals(state2, false)) {
                    // Find any states pointing to state1
                    for (RawState state3 : aut) {
                        if (state3.get_succ().indexOf(state1) != -1) {
                            // Redirect transitions to state2
                            state3.del_succ(state1); 
                            state3.add_succ(state2); 
                            // Mark the extra state for deletion 
                            state1.set_rank(-1);
                        }
                    }
                }
            }

            if (state1.get_rank() == -1) {
                num_removed++;
            }
            
        }

        
        System.out.println("Removed " + num_removed + " stutter states.");
        
        */

        /* Print output */

		
		
		if (det) {
			String res = "";
		
			for (RawState state : aut) {
	            if (state.get_rank() != -1) {
	                res += state + "\n";
	            }
			}
	
			System.out.print("\n\n");
			System.out.print(res);
			// return null; // res;
			System.out.print("\n\n");
			return falsifyEnv;
		} else {
			if (strategy_kind == 3) return result; else return false;
		}
		
	}
	
	//Default deterministic version for backwards compatibility
	public boolean calculate_strategy(int kind, BDD ini) {
		return calculate_strategy(kind, ini, true);
	}
	
	
	/**
	 * <p>
	 * Extracting a safety automaton characterizing all allowed system moves. 
	 * Used to restrict the user during counterstrategy visualization with Mopsy.	 
	 */
	
	public void generate_safety_aut(BDD ini) {
		Stack<BDD> st_stack = new Stack<BDD>();
		Stack<RawState> aut = new Stack<RawState>();

        BDDIterator ini_iterator = ini.iterator(env.moduleUnprimeVars().union(sys.moduleUnprimeVars()));

        while (ini_iterator.hasNext()) {

            BDD this_ini = (BDD) ini_iterator.next();

            RawState test_st = new RawState(aut.size(), this_ini, 0);

            int idx = -1;
            for (RawState cmp_st : aut) {
                if (cmp_st.equals(test_st, false)) { // search ignoring rank
                    idx = aut.indexOf(cmp_st);
                    break;
                }
            }

            if (idx != -1) {
                // This initial state is already in the automaton
                continue;
            }

            // Otherwise, we need to attach this initial state to the automaton

            st_stack.push(this_ini);

            // iterating over the stacks.
            while (!st_stack.isEmpty()) {
                // making a new entry.
                BDD p_st = st_stack.pop();

                /* Create a new automaton state for our current state 
                  (or use a matching one if it already exists) */
                RawState new_state = new RawState(aut.size(), p_st, 0);
                int nidx = aut.indexOf(new_state);
                if (nidx == -1) {
                    aut.push(new_state);
                } else {
                    new_state = aut.elementAt(nidx);
                }

                BDD next_op = Env.unprime(sys.trans().and(p_st).exist(env.moduleUnprimeVars().union(sys.moduleUnprimeVars())));

                BDDIterator next_iterator = next_op.iterator(env.moduleUnprimeVars().union(sys.moduleUnprimeVars()));
                while (next_iterator.hasNext()) {

                    BDD this_next = (BDD) next_iterator.next();
                    //this_next.printSet();
                    RawState gsucc = new RawState(aut.size(), this_next, 0);
                    idx = aut.indexOf(gsucc); // the equals doesn't consider
                                              // the id number.
                    if (idx == -1) {
                        st_stack.push(this_next);
                        aut.add(gsucc);
                        idx = aut.indexOf(gsucc);
                    }
                    new_state.add_succ(aut.elementAt(idx));
                }

                //System.out.print("------------\n");
            }
        }

        /* Print output */

		String res = "";
		for (RawState state : aut) {
            if (state.get_rank() != -1) {
                res += state + "\n";
            }
		}

		System.out.print("\n\n");
		System.out.print(res);
		// return null; // res;
	}

	
       // Compute the symbolic winning strategy
       public BDD[][] calculate_symb_strategy() {

               BDD[][] res;

               // result is an array of BDDs that contains the following strategies:
               // 0. The strategies that do not change the justice pursued
               // 1. The strategies that change the justice pursued
               res = new BDD[2][sys.justiceNum()];

               BDD trans = env.trans().and(sys.trans());
               
               // Initialize the strategies for the case that the justice is visited
               // In this case the memory value is adjusted.
               for (int i=0 ; i<sys.justiceNum(); i++) {
                       int next_just = (i + 1) % sys.justiceNum();
                       res[0][i] = Env.FALSE();
                       res[1][i] = z_mem[i].and(trans.and(sys.justiceAt(i).and(Env.prime(z_mem[next_just]))));
               }

               // Add to the strategy that does not visit the justice the option to reduce the distance to the current goal
               // That is, reduce the index of the y_mem visited.
               for (int i=0 ; i < sys.justiceNum() ; i++) {
                       BDD low = y_mem[i][0];
                       for (int r=1 ; r<y_mem[i].length ; r++) {
                                // look for the minimal y_mem we can jump to
                               //BDD cand = Env.FALSE();
                               // int look_r = 0;
                               //for (int look_r = 0; cand.isZero(); look_r++) {
                                //   cand = y_mem[i][r].and(low.not()).and(trans.and(Env.prime(y_mem[i][look_r])));
                                //}
                                BDD cand = y_mem[i][r].and(low.not()).and(trans.and(Env.prime(low)));
                               res[0][i] = res[0][i].or(cand);
                               low = low.or(y_mem[i][r]);
                       }
               }

               // Add to the strategy that does not visit the justice the option to not visit the justice of the environment and remain in same distance
               // That is, remain in the same x_mem.
               // FIXME: we don't really want this in practical situations
               for (int i=0 ; i < sys.justiceNum() ; i++) {
                       BDD low = y_mem[i][0];
                       for (int r=1 ; r<x_mem[i].length ; r++) {
                               for (int j=0 ; j < env.justiceNum(); j++){
                                       BDD cand = x_mem[i][r][j].and(low.not()).and(trans).and(env.justiceAt(j).not()).and(Env.prime(x_mem[i][r][j]));
                                       res[0][i] = res[0][i].or(cand);
                                       low = low.or(x_mem[i][r][j]);
                               }
                       }
               }

               return res;
       }
       	
		
	/**
	 * <p>
	 Extracting an arbitrary counterstrategy from the set of possible counterstrategies, following the approach outlined in 
	 * </p>
	 * <p>
	 * Robert Konighofer, Georg Hofferek, Roderick Bloem. Debugging formal specifications using simple counterstrategies. In FMCAD 2009, pp. 152-159.
	 * </p>
	 * 
	 * @param enable_234
	 *            Whether moves of all types are to be allowed, or only moves leading closer to a system violation
	 * @param det
	 *            true if a deterministic strategy is desired, otherwise false
	 *            
	 */
	public boolean calculate_counterstrategy(BDD ini, boolean enable_234, boolean det) {
		
		Stack<BDD> st_stack = new Stack<BDD>();
		Stack<Integer> i_stack = new Stack<Integer>();
		Stack<Integer> j_stack = new Stack<Integer>();	
		Stack<RawCState> aut = new Stack<RawCState>();
		boolean result, noDeadlock;
		if (ini.isZero()) result = false; else result = true;
		if (ini.isZero()) noDeadlock = false; else noDeadlock = true;
		
        BDDIterator ini_iterator = ini.iterator(env.moduleUnprimeVars().union(sys.moduleUnprimeVars()));
		
		
        while (ini_iterator.hasNext()) {       
	        int a = 0;
	        BDD this_ini = (BDD) ini_iterator.next();

			int idx = -1;
	        st_stack.push(this_ini);
	        i_stack.push(new Integer(0)); // TODO: is there a better default i?
	        j_stack.push(new Integer(-1)); // TODO: is there a better default j?
	        
	        
	        // iterating over the stacks.
	        while (!st_stack.isEmpty()) {
	        		// making a new entry.
				BDD p_st = st_stack.pop();
				int rank_i = i_stack.pop().intValue();
				int rank_j = j_stack.pop().intValue();
				
				/* Create a new automaton state for our current state 
				* (or use a matching one if it already exists) 
				* p_st is the current state value,
				* rank_i is the environment goal currently being pursued, 
				* and rank_j is the system goal currently being prevented.
				* cf. RawCState class definition below.
				*/
				
				RawCState new_state = new RawCState(aut.size(), p_st, rank_j, rank_i, Env.FALSE());
				int nidx = aut.indexOf(new_state);
				if (nidx == -1) {
					aut.push(new_state);
				} else {
					new_state = aut.elementAt(nidx);
				}		


				int new_i = 0, new_j = -1;
				
				/* Find Z index of current state */
				// find minimal cy and an i
				int p_az = -1;
				for (int i = 0; i < z2_mem.length; i++) {
					if (!p_st.and(z2_mem[i]).isZero()) {
						p_az = i;
						break;
					}
				}
				//System.out.println("p_az "+p_az);
				assert p_az >= 0 : p_st+"Couldn't find p_az";
			   
				/* Find Y index of current state */
				int p_j = -1;
				for (int j = 0; j < sysJustNum; j++) {
					 if (!p_st.and(controlStatesX(env,sys,(y2_mem[j][p_az]))).isZero()) {
						p_j = j;
						break;
					}
				}
				//System.out.println("p_j "+p_j);
				
				assert p_j >= 0 : "Couldn't find p_j";

				
				/* Find X index of current state */
				int p_c = -1;
				for (int c = 0; c < x2_mem[p_j][rank_i][p_az].length; c++) {
					if (!p_st.and(controlStatesX(env,sys,(x2_mem[p_j][rank_i][p_az][c]))).isZero()) {
						p_c = c;
						break;
					}
				}
				//System.out.println("p_c "+p_c);
				assert p_c >= 0 : "Couldn't find p_c";
				
				
				BDD input = Env.FALSE();
				BDD primed_cur_succ = Env.FALSE();
					
				if (det) {
					primed_cur_succ = Env.prime(env.succ(p_st));
				} 
				else {
					primed_cur_succ = Env.TRUE();
				}
				
				input = Env.FALSE();
				
				while (input.isZero()) {
					
					//\rho_1 transitions in K\"onighofer et al
					
					if (p_az == 0) {
						//CHECK IF WE CAN FORCE A SAFETY VIOLATION IN ONE STEP
						input = (p_st.and(primed_cur_succ.and(controlStatesX(env,sys,Env.FALSE())))); //CONSIDERS CURRENT ENV. MOVE ONLY 
						
						
					} else {							
						input = (p_st.and((primed_cur_succ.and((controlStatesX(env,sys,(z2_mem[p_az-1])))) )));
						
					}
					
					if (!input.isZero()) {	
							//System.out.println("RHO 1");
							new_i = rank_i;
							new_j = -1;
						
							if (!enable_234) {
								if (!input.equals(p_st.and(primed_cur_succ)) && !det) {
									return false;
								} 
								continue;
							}
							break;
					}					                
					
					
					//\rho_2 transitions		                
					if (rank_j == -1) {
						//same thing as above -- Z_0 is special
						if (p_az == 0) input = (p_st.and(primed_cur_succ.and(controlStatesX(env,sys,(Env.unprime(primed_cur_succ).and((y2_mem[p_j][p_az])))))).and((controlStatesX(env,sys,Env.FALSE())).not()));
						else input = (p_st.and(primed_cur_succ.and(controlStatesX(env,sys,(Env.unprime(primed_cur_succ).and((y2_mem[p_j][p_az])))))).and((controlStatesX(env,sys,z2_mem[p_az-1])).not()));
						if (!input.isZero()) {
							//System.out.println("RHO 2");									
							new_i = rank_i;
							new_j = p_j;
							break;
						}	
					}
					
					
					//\rho_3 transitions						
					if (rank_j != -1 && rank_i != -1 && !(p_st.and(env.justiceAt(rank_i))).isZero()) {
						if (p_az == 0) 
							input = ((p_st.and(env.justiceAt(rank_i)).and(primed_cur_succ.and(controlStatesX(env,sys,(Env.unprime(primed_cur_succ).and((y2_mem[p_j][p_az]))))))
									.and((controlStatesX(env,sys,(Env.FALSE()))).not())));		        				
						else 
							input = ((p_st.and(env.justiceAt(rank_i)).and(primed_cur_succ.and((controlStatesX(env,sys,(Env.unprime(primed_cur_succ).and((y2_mem[p_j][p_az])))))))
									.and((controlStatesX(env,sys,z2_mem[p_az-1])).not())));
					}
					if (!input.isZero() && det) {	
						//System.out.println("RHO 3");	
						new_i = (rank_i + 1) % env.justiceNum();
						new_j = p_j;		                							
						break;						
					}
					
					
					//\rho_4 transitions: note that you have to lead the transition back to the unfulfiled j in Z_(p_az)
					if (rank_i != -1 && rank_j != -1) {
						if (p_az == 0) {
							if (p_c == 0) {
								input = (((p_st.and((primed_cur_succ.and((controlStatesX(env,sys,(Env.unprime(primed_cur_succ).and(env.justiceAt(rank_i)).and(x2_mem[p_j][rank_i][p_az][p_c]))))))))
												//this next clause is probably superfluous because of \rho_1
												.and((controlStatesX(env,sys,Env.FALSE())).not())));
													
							} else {
								input = (((p_st.and(primed_cur_succ.and((controlStatesX(env,sys,(Env.unprime(primed_cur_succ).and(x2_mem[p_j][rank_i][p_az][p_c-1])))))))
												.and((controlStatesX(env,sys,Env.FALSE())).not())));
							}
						} else {
							if (p_c == 0) {
								input = (((p_st.and(primed_cur_succ.and((controlStatesX(env,sys,(Env.unprime(primed_cur_succ).and(env.justiceAt(rank_i)).and(x2_mem[p_j][rank_i][p_az][p_c])))))))
												.and((controlStatesX(env,sys,z2_mem[p_az-1])).not())));
								
																				
							
							} else {
								input = (((p_st.and(primed_cur_succ.and((controlStatesX(env,sys,(Env.unprime(primed_cur_succ).and(x2_mem[p_j][rank_i][p_az][p_c-1])))))))
												.and((controlStatesX(env,sys,z2_mem[p_az-1])).not())));
													
							}
						}	
							
						if (!input.isZero()) {
							//System.out.println("RHO 4");	
							new_i = rank_i;
							new_j = p_j;
						}
					}
				
					
					if (input.isZero() && !det) return false; 	
				}
				
				
				assert (!(input.isZero() && det)) : p_st + "No successor was found";
				noDeadlock = noDeadlock & addState(new_state, input, new_i, new_j, aut, st_stack, i_stack, j_stack, det);
				result = result & noDeadlock;
				
				
                //when detecting system unsatisfiability, all env actions in primed_cur_succ (which is actualy Env.TRUE() in the nondet case) should be valid
				//result = result & (input.equals(p_st.and(env.trans())));
				//System.out.println("Input= "+input);
				//System.out.println("Possibilities= "+p_st.and(primed_cur_succ));
				result = result & (input.equals(p_st.and(env.trans())));
                if (!result && !det) break;
				//result is true if for every state, all environment actions take us into a lower iterate of Z
				//this means the environment can do anything to prevent the system from achieving some goal.
				
			}

            if (!result) break;
		}
        
                
        if (det) {
        	String res = "";
        
	        for (RawCState state : aut) {
	        	if (state.get_rank_i() != -1) {
	        		res += state + "\n";
	        	}	
	        }	
	        System.out.print("\n\n");
	        System.out.print(res);
        }
        if (!enable_234) {
        	return (!noDeadlock);
        } else {
        	return result;
        }
        
	}
         	

	//Default is deterministic and allows all types of transitions
	public void calculate_counterstrategy(BDD ini) {
		calculate_counterstrategy(ini, true, true);
	}
	
	
	//method for adding stated to the aut and state stack, based on whether we want a deterministic or nondet automaton
 	private boolean addState(RawCState new_state, BDD input, int new_i, int new_j, Stack<RawCState> aut, Stack<BDD> st_stack, Stack<Integer> i_stack, Stack<Integer> j_stack, boolean det) {
    	   boolean noDeadlock = false; //mark state as deadlocked until proven otherwise
    	   for (BDDIterator inputIter = input.iterator(env
                    .modulePrimeVars()); inputIter.hasNext();) {
						
    		   BDD inputOne = (BDD) inputIter.next();
    		   while (det && inputOne.and(env.trans()).isZero() && inputIter.hasNext()) {
    			   inputOne = (BDD) inputIter.next();
    	   		}
    		   if (inputOne.and(env.trans()).isZero()) {
    			   break;
    		   }
    	
    		   
    		    // computing the set of system possible successors.
                Vector<BDD> sys_succs = new Vector<BDD>();      
                BDD all_sys_succs = sys.succ(new_state.get_state().and(inputOne));

                int idx = -1;
               
                
                if (all_sys_succs.equals(Env.FALSE())) {
					RawCState gsucc = new RawCState(aut.size(), Env.unprime(inputOne), new_j, new_i, inputOne);
					idx = aut.indexOf(gsucc); // the equals doesn't consider
                                              // the id number.
                    if (idx == -1) {                       
                        aut.add(gsucc);
                        idx = aut.indexOf(gsucc);
                    }
                    new_state.add_succ(aut.elementAt(idx));                   
					
                	if (det) break; else continue;                	
                }               	
                
				
                for (BDDIterator all_sys_states = all_sys_succs.iterator(sys
                        .moduleUnprimeVars().union(env
                        .moduleUnprimeVars())); all_sys_states.hasNext();) {
                    BDD sin = (BDD) all_sys_states.next();
                    sys_succs.add(sin);
                }
                
                

                // For each system successor, find a strategy successor
                for (Iterator<BDD> iter_succ = sys_succs.iterator(); iter_succ
                        .hasNext();) {
                    BDD sys_succ = iter_succ.next().and(Env.unprime(inputOne));
					
                    //Make sure this is a safe successor state
					if (!sys_succ.and(sys.trans()).isZero()) {
						//the state is not deadlocked
						noDeadlock = true;
	           		    
						RawCState gsucc = new RawCState(aut.size(), sys_succ, new_j, new_i, inputOne);
						idx = aut.indexOf(gsucc); // the equals doesn't consider
												  // the id number.
						if (idx == -1) {
							//System.out.println("Adding system successor = " + sys_succ + " of " + new_state.get_state());
							   
							st_stack.push(sys_succ);
							i_stack.push(new_i);
							j_stack.push(new_j);
							aut.add(gsucc);
							idx = aut.indexOf(gsucc);
						}
						new_state.add_succ(aut.elementAt(idx));						
					} 
                }
                if (det) break;
    	 	}
    	   return noDeadlock;
     }
		
	@SuppressWarnings("unused")
	//Class for a state of the STRATEGY automaton. 
	//The "rank" is the system goal currently being pursued.
	private class RawState {
		private int id;
		private int rank;
		private BDD state;
		private Vector<RawState> succ;

		public RawState(int id, BDD state, int rank) {
			this.id = id;
			this.state = state;
			this.rank = rank;
			succ = new Vector<RawState>(10);
		}

		public void add_succ(RawState to_add) {
			succ.add(to_add);
		}

		public void del_succ(RawState to_del) {
			succ.remove(to_del);
		}

		public BDD get_state() {
			return this.state;
		}

		public int get_rank() {
			return this.rank;
		}

		public void set_rank(int rank) {
			this.rank = rank;
		}

		public Vector<RawState> get_succ() {
			//RawState[] res = new RawState[this.succ.size()];
			//this.succ.toArray(res);
			return this.succ;
		}

		public boolean equals(Object other) {
            return this.equals(other, true);
        }

		public boolean equals(Object other, boolean use_rank) {
			if (!(other instanceof RawState))
				return false;
			RawState other_raw = (RawState) other;
			if (other_raw == null)
				return false;

            if (use_rank) {
                return ((this.rank == other_raw.rank) & (this.state
                        .equals(other_raw.state)));
            } else {
                return (this.state.equals(other_raw.state));
            }
		}

		public String toString() {
			String res = "State " + id + " with rank " + rank + " -> "
					+ state.toStringWithDomains(Env.stringer) + "\n";
			if (succ.isEmpty()) {
				res += "\tWith no successors.";
			} else {
				RawState[] all_succ = new RawState[succ.size()];
				succ.toArray(all_succ);
				res += "\tWith successors : " + all_succ[0].id;
				for (int i = 1; i < all_succ.length; i++) {
					res += ", " + all_succ[i].id;
				}
			}
			return res;
		}
	}
	
	
	//Class for a state of the COUNTERSTRATEGY automaton.  
	//"rank_i" is the environment goal currently being pursued, 
	//and "rank_j" is the system goal currently being prevented.
	private class RawCState {
		private int id;
		private int rank_i;
		private int rank_j;
		private BDD input;
		private BDD state;
		private Vector<RawCState> succ;		
		
		
		public RawCState(int id, BDD state, int rank_j, int rank_i, BDD input) {
			this.id = id;
			this.state = state;
			this.rank_i = rank_i;
			this.rank_j = rank_j;
			this.input = input;
			succ = new Vector<RawCState>(10);
		}

		public void add_succ(RawCState to_add) {
			succ.add(to_add);
		}

		public void del_succ(RawCState to_del) {
			succ.remove(to_del);
		}


		public BDD get_input() {
			return this.input;
		}

		public void set_input(BDD input) {
			this.input = input;
		}

		public BDD get_state() {
			return this.state;
		}

		public int get_rank_i() {
			return this.rank_i;
		}

		public void set_rank_i(int rank) {
			this.rank_i = rank;
		}
		
		public int get_rank_j() {
			return this.rank_j;
		}

		public void set_rank_j(int rank) {
			this.rank_j = rank;
		}


		public boolean equals(Object other) {
            return this.equals(other, true);
        }

		public boolean equals(Object other, boolean use_rank) {
			if (!(other instanceof RawCState))
				return false;
			RawCState other_raw = (RawCState) other;
			if (other_raw == null)
				return false;
            if (use_rank) {
                return ((this.rank_i == other_raw.rank_i) & (this.rank_j == other_raw.rank_j) &
                		(this.state.equals(other_raw.state)));
            } else {
                return ((this.state.equals(other_raw.state)));
            }
		}

			
		public String toString() {
			String res = "State " + id + " with rank (" + rank_i + "," + rank_j + ") -> " 
				+ state.toStringWithDomains(Env.stringer) + "\n";
			if (succ.isEmpty()) {
				res += "\tWith no successors.";				
			} else {
				RawCState[] all_succ = new RawCState[succ.size()];
				succ.toArray(all_succ);
				res += "\tWith successors : " + all_succ[0].id;
				for (int i = 1; i < all_succ.length; i++) {
					res += ", " + all_succ[i].id;
				}
			}
			return res;
		}
	}

	/**
	 * <p>
	 * Getter for the environment player.
	 * </p>
	 * 
	 * @return The environment player.
	 */
	public ModuleWithWeakFairness getEnvPlayer() {
		return env;
	}

	/**
	 * <p>
	 * Getter for the system player.
	 * </p>
	 * 
	 * @return The system player.
	 */
	public ModuleWithWeakFairness getSysPlayer() {
		return sys;
	}

	/**
	 * <p>
	 * Getter for the environment's winning states.
	 * </p>
	 * 
	 * @return The environment's winning states.
	 */
	public BDD sysWinningStates() {
		return player2_winning;
	}

	/**
	 * <p>
	 * Getter for the system's winning states.
	 * </p>
	 * 
	 * @return The system's winning states.
	 */
	public BDD envWinningStates() {
		return player2_winning.not();
	}

	public BDD gameInitials() {
		return getSysPlayer().initial().and(getEnvPlayer().initial());
	}

	public BDD[] playersWinningStates() {
		return new BDD[] { envWinningStates(), sysWinningStates() };
	}

	public BDD firstPlayersWinningStates() {
		return envWinningStates();
	}

	public BDD secondPlayersWinningStates() {
		return sysWinningStates();
	}

}
