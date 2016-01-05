'''
Provides utilities for unrealizable core-finding functionality

'''

import math, re, sys, random, os, subprocess, time
from copy import copy, deepcopy
from logic import to_cnf
from multiprocessing import Pool
import threading
import itertools
import logging
import random


''' UTILS FOR CONVERTING TO AND FROM DIMACS FORMAT '''

def conjunctsToCNF(conjuncts, propList):
    '''
    takes a list of LTL conjuncts constituting the specification ('conjuncts')
    and a list of propositions ('propList') used in them,
    and converts them into DIMACS CNF format replacing each proposition
    with its index in the list. 
    returns:
             mapping: a mapping from LTL formulas to CNF clause numbers
             cnfMapping: a mapping from LTL formulas to their CNF form
             cnfClauses: CNFs corresponding to initial and transition formulas
                         (represents one-step unrolling)
             transClauses: CNFS corresponding to transition formulas
                         (useful for further unrolling later)
             goalClauses: CNFS corresponding to goal formulas
                         (useful for checking goals at each time step)
    '''
    
    #create new variables for 'primed' instances
    propListNext = map(lambda s: 'next_'+s, propList)
    
    #map propositions (in time steps 0 and 1) to numbers for use with SAT solver, i.e. DIMACS format
    props = {propList[x]:x+1 for x in range(0,len(propList))}
    propsNext = {propListNext[x]:len(propList)+x+1 for x in range(0,len(propListNext))}
    
    print "Variable mapping: ",props
    
    #initialize mapping to empty list for each conjunct
    mapping = {conjuncts[x]:[] for x in range(0,len(conjuncts))}
    
    #initialization
    cnfClauses = []
    transClauses = []
    goalClauses = []
    n = 0 #counts number of clauses generated for mapping LTL conjuncts to clause indices
    
    #convert conjuncts to CNF
    allCnfs = runMap(lineToCnf, conjuncts)
    
    #associate original LTL conjuncts with newly created CNF clauses
    cnfMapping = {line:cnf.split("&") for cnf, line in zip(allCnfs,conjuncts) if cnf}
    
    
    #create DIMACS-form CNF clauses from the above CNF formulas by replacng props with numbers
    #map original LTL conjuncts to DIMACS clause indices (without unrolling)
    for cnf, lineOld in zip(allCnfs,conjuncts):     
      if cnf: 
        allClauses = cnf.split("&");
        for clause in allClauses:    
            #replace prop names with var numbers at depth 0
            clause = replaceProps(clause, 0, props, propsNext)
            
            #append clauses to appropriate list  
            if "<>" in lineOld:
                goalClauses.append(clause)
            elif "[]" in lineOld:
                transClauses.append(clause)
                cnfClauses.append(clause)                                
            else:
                cnfClauses.append(clause)         
            
        if not "<>" in lineOld:
            #for non-goal (i.e. trans and init) formulas, extend mapping with line nos.
            #the guilty goal is always put last, so we don't need the clause nos.
            #increment n with the number of newly generated (non-goal) clauses
            mapping[lineOld].extend(range(n+1,n+1+len(allClauses)))    
            n = n + len(allClauses)
    
    return mapping, cnfMapping, cnfClauses, transClauses, goalClauses

def replaceProps(clause, depth, props, propsNext):
    #replace prop names with var numbers at the specified depth. Do 'next' first.
    for k in propsNext.keys():
        clause = re.sub("\\b"+k+"\\b",str(propsNext[k]+depth*len(props)), clause)
    for k in props.keys():
        clause = re.sub("\\b"+k+"\\b",str(props[k]+depth*len(props)), clause)   
    #add trailing 0   
    clause = clause.strip()+" 0\n"
    return clause

def cnfToConjuncts(cnfIndices, mapping, cnfMapping, input):
    '''
    convert a list of DIMACS CNF clause indices ('cnfIndices') to
    the corresponding LTL conjuncts (reverse lookup in 'mapping')
    '''
    conjuncts = []
    for k in mapping.keys(): #for each conjunct
        #if not set([input[i] for i in mapping[k]]).isdisjoint([input[i] for i in cnfIndices]): #if some DIMACS clause in cnfIndices came from this conjunct
            #print [cnfMapping[k][i%len(cnfMapping[k])] for i in mapping[k] if i in cnfIndices]
#            print [d for d in zip(mapping[k],cnfMapping[k]) if d[0] in cnfIndices]
        if not set(mapping[k]).isdisjoint(cnfIndices): #if some DIMACS clause in cnfIndices came from this conjunct
            print "from conjunct ",k
            intersection = set(mapping[k]).intersection(cnfIndices)
            indices = [mapping[k].index(i) for i in intersection]
            for i in indices:
            #for i in range(len(mapping[k])):
                print cnfMapping[k][i%len(cnfMapping[k])], ' at time step ', i/len(cnfMapping[k])
                #if mapping[k][i] in set(mapping[k]).intersection(cnfIndices): #find the specific DIMACS clause
                    #print cnfMapping[k][i%len(cnfMapping[k])], ' at time step ', i/len(cnfMapping[k]) #deduce corresponding CNF clause and time step
            conjuncts.append(k)  
            #print k , (set(mapping[k]).intersection(cnfIndices))
    return conjuncts

    
def makeHumanReadableCNFFromIndices(indices, input_data, ignoreBound, numProps):
    '''
        useful for debugging the input to the SAT solver
        indices: clause indices to be displayed from input
        input_data: input to SAT solver (in DIMACS format)
        ignoreBound: clause indices lower than this are ignored
        numProps: used to get variable range
    '''
    def cnfVar2LTLVar(m):
        props = range(0,numProps)
        negated = (m.group("sign") == "-")
        num = int(m.group("num"))

        if num == 0:
            return ""

        time_step = int(math.floor((num - 1)/len(props)))
        var_num = (num - len(props)*time_step)
        
        name = "{}@{}".format(var_num, time_step)
        if negated:
            name = "~"+name
        return name
        
    output = []
    for input_line_num in indices: 
        cnf_clause = input_data[input_line_num]
        cnf_clause = re.sub(r"(?P<sign>-)?(?P<num>\d+)", cnfVar2LTLVar, cnf_clause)
        if input_line_num <= ignoreBound:
            cnf_clause = cnf_clause.rstrip() + " (ignored)\n"
        output.append(cnf_clause)

    return output


                

        
def findGuiltyLTLConjuncts(cmd, depth, numProps, init, trans, goals, mapping,  cnfMapping, conjuncts, maxDepth, ignoreDepth, cyc_enc=True,
                           timing=False): 
        #returns the ltl conjuncts returned as an unsat core when unrolling trans depth times from init and 
        #checking goal at final time step
        #note that init contains one-step unrolling of trans already
        
        #if timing: 
        #    cmd = ["time"] +cmd
        
            
        mapping = deepcopy(mapping)
        #precompute p and n
        p = (maxDepth+2)*(numProps)
        #the +2 is because init contains one trans already 
        #(so effectively there are depth+1 time steps and one final "next" time step)        
        
        n = (depth)*(len(trans)) + len(init) + len(goals) #counts number of clauses generated for mapping LTL to line numbers
    
        if cyc_enc:
            cycleInds = map(lambda x: int(x), re.findall(r'cycle([0-9]+)', "".join(init)))
            if cycleInds:
                numCycles = max(cycleInds)+1
            else:
                numCycles = 0
            for i in reversed(range(numCycles)):
                init = map(lambda s: s.replace("cycle"+str(i), str(p+i+1)),init)
                #print "Cycle: ",str(i), "Variable: ",str(p+i+1)
            #additional variables for cycles
            p = p + numCycles
            
            print "p",p,"numCycles",numCycles
        
        
        if ignoreDepth == 0:
            ignoreBound = 0
        else:
            ignoreBound = len(init) + (ignoreDepth)*len(trans)
            
        output = []
        
        #find minimal unsatisfiable core by calling picomus
        if cmd is None:
            return (False, False, [], "")   
                
        #start a reader thread        
        start = time.time()
        subp = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)                                            
        readThread =  threading.Thread(target = subprocessReadThread, args=(subp.stdout,output))
        readThread.daemon = True
        readThread.start()
        
        
        #send header
        input = ["p cnf "+str(p)+" "+str(n)+"\n"]
        subp.stdin.write(input[0])
        subp.stdin.writelines(init)
        
        input.extend(init)
        
        #Duplicating transition clauses for depth greater than 1         
        numOrigClauses = len(trans)  
        #the depth tells you how many time steps of trans to use
        #depth 0 just checks init with goals

        for i in range(1,depth+1):
            for clause in trans:
                newClause = ""
                for c in clause.split():
                    intC = int(c)
                    newClause= newClause + str(cmp(intC,0)*(abs(intC)+numProps*i)) +" "                            
                newClause=newClause+"\n"
                #send this clause
                subp.stdin.write(newClause)
                input.append(newClause)
                
                
            j = 0    
            for line in conjuncts:
                #update mapping with newly added trans clause line numbers
                if "[]" in line and "<>" not in line:                      
                    numVarsInTrans = (len(mapping[line]))/(i+1)
                    mapping[line].extend(map(lambda x: x+numOrigClauses, mapping[line][-numVarsInTrans:]))
                    j = j + 1
                    
        
        #create goal clauses
        dg = map(lambda x: ' '.join(map(lambda y: str(cmp(int(y),0)*(abs(int(y))+numProps*(depth))), x.split())) + '\n', goals)
        
        print "GOAL DEPTH",depth
        print "GOALS",dg
        
        #send goalClauses
        subp.stdin.writelines(dg)
        input.extend(dg)

        
        #send EOF
        subp.stdin.close()
        
        #update mapping with newly added goal clause line numbers
        nMinusG = n - len(goals)
        for line in conjuncts:
            if "<>" in line:
                mapping[line] = range(nMinusG+1,nMinusG+len(goals)+1)
       
        readThread.join()
                
        sys.stdout = sys.__stdout__
        sys.stderr = sys.__stderr__

#        #Write output to file (mainly for debugging purposes)
#        satFileName = "debug"+str(random.randint(0,1000))+".sat"
#        outputFile = open(satFileName,'w')
#        outputFile.write("\n".join(output))
#        logging.debug("wrote {}".format(satFileName))
#        outputFile.close()
#        
#        #Write input to file (mainly for debugging purposes)
#        satFileName = "input"+str(random.randint(0,1000))+".sat"
#        inputFile = open(satFileName,'w')
#        inputFile.write("\n".join(input))
#        logging.debug("wrote {}".format(satFileName))
#        inputFile.close()
            
        if timing:
            print 'ELAPSED TIME:',time.time()-start
        
        if any(["WARNING: core extraction disabled" in s for s in output]):
            # never again
            logging.error("************************************************")
            logging.error("*** ERROR: picomus needs to be compiled with ***")
            logging.error("*** trace support, or things will misbehave. ***")
            logging.error("***                                          ***")
            logging.error("*** Recompile with ./configure --trace       ***")
            logging.error("************************************************")
            return []

        #if timing & any(["real" in s for s in output]):
        #    logging.info([l for l in output if "real" in l])
        
        if any(["UNSATISFIABLE" in s for s in output]):
            logging.info("Unsatisfiable core found at depth {}".format(depth))
        elif any(["SATISFIABLE" in s for s in output]):
            logging.info("Satisfiable at depth {}".format(depth))
            if depth==maxDepth:
                print output
            return []
        else:
            logging.error("Picosat error: {!r}".format(output))
        
        """cnfIndices = []
        for line in output.split('\n'):
                if re.match('^v', line):
                    index = int(line.strip('v').strip())
                    if index!=0:
                        cnfIndices.append(index)
            """
            
        
        #pythonified the above
        #get indices of contributing clauses
        cnfIndices = filter(lambda y: y!=0, map((lambda x: int(x.strip('v').strip())), filter(lambda z: re.match('^v', z), output)))
        print cnfIndices
        print "GUILTY CLAUSES: ",[input[n] for n in cnfIndices]
        
        #ignoreBound = 0
        #get corresponding LTL conjuncts
        #guilty = cnfToConjuncts([idx for idx in cnfIndices if idx > ignoreBound], mapping, cnfMapping, input)
        guilty = cnfToConjuncts(cnfIndices, mapping, cnfMapping, input)
        
        
        print makeHumanReadableCNFFromIndices(map(lambda c: int(c), cnfIndices), input, 0, numProps)    
        
        return guilty
    
        
def unsatCoreCases(cmd, propList, topo, badInit, conjuncts, maxDepth, initDepth, extra=[], cyc_enc=True):
    
        
     #returns the minimal unsatisfiable core (LTL formulas) given
     #        cmd: picosat command
     #        propList: list of proposition names used
     #        topo: LTL formula describing topology
     #        badInit: formula describing bad initial states
     #        conjuncts: remaining LTL formulas highlighted by preliminary analysis
     #        maxDepth: determines how many time steps we unroll 
     #        initDepth: used to determine minimum depth to prevent false alarms (every depth between initDepth and maxDepth is checked)
       
        numProps = len(propList)
        
        if extra and not isinstance(extra[0], basestring):
            extra = [x for e in extra for x in e]
        
        #first try without topo and init, see if it is satisfiable
        ignoreDepth = 0    
        mapping, cnfMapping, init, trans, goals = conjunctsToCNF(conjuncts, propList)
                      
        
        init.extend(extra)
        
        logging.info("Trying to find core without topo or init") 

        #for each depth, find the conjuncts that prevent the goal(if anhy)
        guiltyList = runMap(findGuiltyLTLConjunctsWrapper, itertools.izip(itertools.repeat(cmd),
                                                                          range(initDepth, maxDepth+1),
                                                                          itertools.repeat(numProps),
                                                                          itertools.repeat(init),
                                                                          itertools.repeat(trans), 
                                                                          itertools.repeat(goals),
                                                                          itertools.repeat(mapping),
                                                                          itertools.repeat(cnfMapping),
                                                                          itertools.repeat(conjuncts),
                                                                          itertools.repeat(maxDepth),
                                                                          itertools.repeat(ignoreDepth),
                                                                          itertools.repeat(cyc_enc),
                                                                          itertools.repeat(True)))

        #allGuilty = map((lambda (depth, cnfs): self.guiltyParallel(depth+1, cnfs, mapping)), list(enumerate(allCnfs)))
        
            
        if all(guiltyList): # goal unsat at all depths
            # return the set of all guilty conjuncts (across all depths)    
            allGuilty = set([item for sublist in guiltyList for item in sublist])
            logging.info("Unsat core found without topo or init")
            return trans, allGuilty
        else:
            # ignoreDepth = len([g for g in guiltyList if g])
            ignoreDepth += next((i for i, x in enumerate(guiltyList) if x), 0) #find first unsat depth
        
        logging.info("ignore depth {}".format(ignoreDepth))
        
            
        #then try just topo and init and see if it is unsatisfiable. If so, return core.
        logging.info("Trying to find core with just topo and init")
        mapping,  cnfMapping, init, trans, goals = conjunctsToCNF([badInit,topo], propList)
       
        guilty = findGuiltyLTLConjuncts(cmd,maxDepth,numProps,init,trans,goals,mapping,cnfMapping,[badInit,topo],maxDepth,0,cyc_enc,True)
        
                #allGuilty = map((lambda (depth, cnfs): self.guiltyParallel(depth+1, cnfs, mapping)), list(enumerate(allCnfs)))
            #print "ENDING PICO MAP"
 
        if guilty:
            logging.info("Unsat core found with just topo and init")
            return trans, guilty
        
        
        #if the problem is in conjunction with the topo but not just topo, keep increasing the depth until something more than just topo is returned
        mapping,  cnfMapping, init, trans, goals = conjunctsToCNF([badInit,topo] + conjuncts, propList)

        init.extend(extra)
        
        logging.info("Trying to find core with everything")
        
        #for liveness, initial depth is set to at least the number of regions.
        # This ensures that we unroll as far as needed to physically get to the goal
        
        # don't use ignoreDepth for deadlock
        if len(goals) == 0:
           ignoreDepth = 0
        else:
            ignoreDepth = max(ignoreDepth, initDepth)
            
        
        guiltyList = runMap(findGuiltyLTLConjunctsWrapper, itertools.izip(itertools.repeat(cmd),
                                                                          range(maxDepth, maxDepth+1),
                                                                          itertools.repeat(numProps),
                                                                          itertools.repeat(init),
                                                                          itertools.repeat(trans),
                                                                          itertools.repeat(goals),
                                                                          itertools.repeat(mapping),
                                                                          itertools.repeat(cnfMapping),
                                                                          itertools.repeat([badInit,topo] + conjuncts),
                                                                          itertools.repeat(maxDepth),
                                                                          itertools.repeat(ignoreDepth),
                                                                          itertools.repeat(cyc_enc),
                                                                          itertools.repeat(True)))

        #allGuilty = map((lambda (depth, cnfs): self.guiltyParallel(depth+1, cnfs, mapping)), list(enumerate(allCnfs)))
        
        guilty = [item for sublist in guiltyList for item in sublist]        
        
        guiltyMinusGoal = [g for g in guilty if '<>' not in g]
  
                        
        justTopo = set([topo, badInit]).issuperset(guiltyMinusGoal)
        
        #while justTopo and depth < maxDepth:
            
            #guilty = findGuiltyLTLConjuncts(cmd,depth,numProps,init,trans,goals,mapping,cnfMapping,[topo, badInit]+conjuncts, ignoreDepth)
            ##allGuilty = map((lambda (depth, cnfs): self.guiltyParallel(depth+1, cnfs, mapping)), list(enumerate(allCnfs)))
            ##print "ENDING PICO MAP"
            
            
            #guiltyMinusGoal = [g for g in guilty if '<>' not in g]
            #if not set([topo, badInit]).issuperset(set(guiltyMinusGoal)):
                #justTopo = False
            #else:
                #depth+=1
            ##get contributing conjuncts from CNF indices            
            ##guilty = cnfToConjuncts(allIndices, mapping)
        
        if guilty:
            logging.info("Unsat core found with all parts")
        else:
            logging.info("Unsat core not found")

        return trans, guilty
    
def findGuiltyLTLConjunctsWrapper(x):        
    return findGuiltyLTLConjuncts(*x)
    
 
def unsatCoreCasesWrapper(x): 
    return unsatCoreCases(*x) 
    

''' CONVERTING STATES AND STATE CYCLES TO LTL '''

def stateToLTL(state, useEnv=1, useSys=1, use_next=False):
    # deprecated -- functionality now provided by State object method getLTLRepresentation()
        def decorate_prop(prop, polarity):
            if int(polarity) == 0:
                prop = "!"+prop
            if use_next:
                prop = "next({})".format(prop)
            return prop
    
        sys_state = " & ".join([decorate_prop("s."+p, v) for p,v in state.inputs.iteritems()])
        env_state = " & ".join([decorate_prop("e."+p, v) for p,v in state.outputs.iteritems()])
                
        if useEnv:
            if useSys:
                return env_state + " & " + sys_state
            else:
                return env_state
        elif useSys:
            return sys_state
        else:
            return ""
            

def stateCycleToCNFs(index, cycle, propList, depth, cyc_enc=True): 
    #expanding the cycle to the required depth and converting to DIMACS

    depth = max(depth, len(cycle))+1
    
    propListNext = map(lambda s: 'next_'+s, propList)
    
    props = {propList[x]:x+1 for x in range(0,len(propList))}
    propsNext = {propListNext[x]:len(propList)+x+1 for x in range(0,len(propListNext))}
    
    #next_cycle = cycle[1:]+[cycle[0]]
    #stateList = ['('+(s1.getLTLRepresentation(swap_players=True, include_inputs=True)+') -> ('+s2.getLTLRepresentation(use_next=True, swap_players=True, include_inputs=False))+')' for (s1,s2) in zip(cycle,next_cycle)]
    if cyc_enc:
        stateList = ["!cycle"+str(index) + " | (" + s1.getLTLRepresentation(swap_players=True, include_inputs=True) + ")" for s1 in cycle]
        initialState = lineToCnf("!cycle"+str(index) + " | ("+cycle[0].getLTLRepresentation(swap_players=True)+ ")")
        d = 0
    else:
        stateList = [s.getLTLRepresentation(use_next = True, swap_players=True, include_inputs=False) for s in cycle]
        initialState = formatForDimacs(formatForToCnf(stripPrefixes(cycle[0].getLTLRepresentation(swap_players=True)+ ")"))) #state LTL rep is already in CNF
        d = 1
        
    stateCnfs = [lineToCnf(s) for s in stateList]
    #initialState = formatForDimacs(formatForToCnf(stripPrefixes("!cycle"+str(index) + " | ("+cycle[0].getLTLRepresentation(swap_players=True)+ ")"))) #state LTL rep is already in CNF
    extra=[];
    #extra = [[replaceProps(clause, 0, props, propsNext) for clause in initialState.split("&")]]
    
    #print "CYCLE "+str(index)+": ", "depth of unrolling: ",depth
    
    while d < depth:
      tempD = d
      for index, line in enumerate(stateCnfs, d%len(stateCnfs)):
        extra.append([replaceProps(clause,tempD,props,propsNext) for clause in line.split("&")])
        tempD = tempD + 1
        if tempD == depth:
            break
      d = tempD
      
    #finalS = cycle[d%len(cycle)]
    #finalEnv = lineToCnf(finalS.getLTLRepresentation(swap_players=True, include_inputs=False))
    #extra.append([replaceProps(clause,d,props,propsNext) for clause in finalEnv.split("&")])
        
    
    #print "EXTRA from cycle "+str(index),extra 
    
    #extra.append([replaceProps(clause,len(stateCnfs)) for clause in initialState.split("&")])
    return extra


def unwindSCCs(SCCs, propList, depth):
    #expanding the SCC to the required depth and converting to DIMACS
    
    #use the state with the lowest index/name as the initial state
    if SCCs:
        initState = SCCs[0]
    initStateName = min([int(s[0].getName()) for s in SCCs])
    for (s,t) in SCCs:
        if int(s.getName()) == initStateName:
            initState = s
            break
        
    
    propListNext = map(lambda s: 'next_'+s, propList)
    props = {propList[x]:x+1 for x in range(0,len(propList))}
    propsNext = {propListNext[x]:len(propList)+x+1 for x in range(0,len(propListNext))}
    
    def getTransitionLTLReps(s,t):
        ltl = "("+s.getLTLRepresentation(swap_players=True) + ") -> (" + t.getLTLRepresentation(use_next = True, swap_players=True,include_inputs = True)+")"
        return lineToCnf(ltl)
    
    initialStateRep = formatForDimacs(formatForToCnf(stripPrefixes(initState.getLTLRepresentation(swap_players=True)))) #state LTL rep is already in CNF
    
    extra = [[replaceProps(clause, 0, props, propsNext) for clause in initialStateRep.split("&")]]
    
    d = 0
    
    print "UNROLL SCC DEPTH",depth
    
    currStates = [(s,t) for (s,t) in SCCs if int(s.getName()) == initStateName]
    for (s,t) in currStates:
        extra.append([replaceProps(clause,0,props,propsNext) for clause in getTransitionLTLReps(s,t).split("&")])
        
    def disjunctStatesLTL(pairs):
        ltl = "("+pairs[0][0].getLTLRepresentation(swap_players=True) + ")"
        for s in pairs[1:]:
            ltl = ltl + " | (" + s[0].getLTLRepresentation(swap_players=True) + ")"
        return lineToCnf(ltl)
    
    while d < depth+1:
        newInits = [s2 for (s1,s2) in currStates]
        newCurrStates = [(s2,s3) for (s2,s3) in SCCs if s2 in newInits]
        for (s,t) in newCurrStates:
            extra.append([replaceProps(clause,d,props,propsNext) for clause in getTransitionLTLReps(s,t).split("&")])
        #validStates = disjunctStatesLTL(newCurrStates).split("&")
        #extra.append([replaceProps(clause,d,props,propsNext) for clause in validStates])
        currStates = newCurrStates
        d+=1
        
    print "EXTRA",extra 
    
    return extra


'''MULTIPROCESSING UTILS'''

USE_MULTIPROCESSING = False

def runMap(function, inputs):
    """ Wrapper for single- and multi-threaded versions of map, to make
        it easy to disable multiprocessing for debugging purposes
    """

    logging.debug("Starting map ({}-threaded): {}".\
                  format("multi" if USE_MULTIPROCESSING else "single", function.__name__))

    if USE_MULTIPROCESSING:
        pool = Pool()
        outputs = pool.map(function, inputs, chunksize = 1)   
        pool.terminate()
    else:
        outputs = map(function, inputs)   

    logging.debug("Finished map: {}".format(function.__name__))

    return outputs

def subprocessReadThread(fd, out):
    for line in fd:
        out.append(line)
        if "expected" in line:
            logging.error(line)
            
''' FORMATTING UTILS '''

def formatForDimacs(clause):
    '''
    syntactic substitutions for format compatibility
    '''
    clause = re.sub('[()]', '', clause)   
    clause = re.sub('[|]', '', clause)           
    clause = re.sub('~', '-', clause) 
    return clause

def stripPrefixes(line):
    '''
    stripping 'e.' and 's.' prefix, replacing next() with next_
    '''
    line = re.sub('s\.','',line)
    line = re.sub('e\.','',line)   
    line = re.sub(r'(next\(\s*!)', r'(!next_', line)         
    line = re.sub(r'(next\(\s*)', r'(next_', line)
    return line


def formatForToCnf(line):
    '''
    syntactic substitutions to work with to_cnf function
    TODO: modify to_cnf instead
    '''
    line = re.sub('!', '~', line)
    line = re.sub('[\s]+', ' ', line)        
    line = re.sub('\<-\>', '<=>', line)
    line = re.sub('->', '>>', line)
    line = line.strip() 
    return line

        
def lineToCnf(line):
        '''
        converts a single LTL formula into CNF form compatible with DIMACS
        '''
        line = stripLTLLine(line)
        if line!='':
            line = stripPrefixes(line)
            line = formatForToCnf(line)
            cnf = str(to_cnf(line))
            cnf = formatForDimacs(cnf)
            return cnf
        else:
            return None        

    
def stripLTLLine(line, useNext=False):
        '''
        strip white text and LTL operators
        adds 'next' marker if flag is true
        '''
        line = re.sub('[\t\n]*','',line)    
        line = re.sub('\<\>','',line)  
        line = re.sub('\[\]','',line)  
        line = line.strip()
        #remove any trailing '&'s
        line = re.sub('&\s*$','',line)   
        if useNext:
            line = re.sub('s\.','next_s.',line)
            line = re.sub('e\.','next_e.',line)                     
        return line
        
