import sys
import re
import time
import pycudd
import strategy
import logging

# NOTE: This module requires a modified version of pycudd!!
# See src/etc/patches/README_PYCUDD for instructions.

# TODO: We could probably get away with a really minimal Python BDD
#       implementation

# TODO: move generic bdd functions to a different file so others can use?
# TODO: variable reordering so Jx and strat_type are first (is this actually
#       best?)

# TODO: optimize strategy:
#       - stutter state removal
#       - minimal Y after Z change

class BDDStrategy(strategy.Strategy):
    def __init__(self, add):
        super(BDDStrategy, self).__init__()

        # We will have a state collection just in order to provide a context
        # for states (FIXME?)
        self.states = strategy.StateCollection()

        self.strategy = None

        self.var_name_to_BDD = {}
        self.BDD_to_var_name = {}

        self.strat_type_var = None

        self.mgr = pycudd.DdManager()
        self.mgr.SetDefault()
        # TODO: why is garbage collection crashing?? :( [e.g. on firefighting]
        self.mgr.DisableGarbageCollection()

        # track if this is an add or bdd
        self.add = add

    def _loadFromFile(self, filename):
        """
        Load in a strategy BDD from a file produced by a synthesizer,
        such as JTLV or Slugs.
        """

        # Clear any existing states
        self.states.clearStates()

        a = pycudd.DdArray(1)

        # Load in the actual BDD or ADD itself
        if self.add:
            # Note: We are using an ADD loader because the BDD loader
            # would expect us to have a reduced BDD with only one leaf node
            self.mgr.AddArrayLoad(pycudd.DDDMP_ROOT_MATCHLIST,
                                  None,
                                  pycudd.DDDMP_VAR_MATCHIDS,
                                  None,
                                  None,
                                  None,
                                  pycudd.DDDMP_MODE_TEXT,
                                  filename, None, a)

            # Convert from a binary (0/1) ADD to a BDD
            self.strategy = self.mgr.addBddPattern(a[0])
            logging.debug('ADD loaded')
        else:
            #try loading as BDD (input from SLUGS) instead of ADD (input from JTLV)
            self.strategy  = self.mgr.BddLoad(pycudd.DDDMP_VAR_MATCHIDS,
                                  None,
                                  None,
                                  None,
                                  pycudd.DDDMP_MODE_TEXT,
                                  filename, None)
            logging.debug('BDD loaded')

        # Load in meta-data
        with open(filename, 'r') as f:
            # Seek forward to the max goal ID notation
            line = ""
            while not line.startswith("# Num goals:"):
                line = f.readline()

            self.num_goals = int(line.split(":")[1])

            # Seek forward to the start of the variable definition section
            while not line.startswith("# Variable names:"):
                line = f.readline()

            # Parse the variable definitions
            for line in f:
                m = re.match(r"^#\s*(?P<num>\d+)\s*:\s*(?P<name>\w+'?)", line)

                # We will stop parsing as soon as we encounter an invalid line
                # Note: This includes empty lines!
                if m is None:
                    break

                varname = m.group("name")
                varnum = int(m.group("num"))

                #### TEMPORARY HACK: REMOVE ME AFTER OTHER COMPONENTS ARE UPDATED!!!
                # Rewrite proposition names to make the old bitvector system work
                # with the new one
                varname = re.sub(r"^bit(\d+)('?)$", r'region_b\1\2', varname)
                #################################################################

                if varname == "strat_type":
                    self.strat_type_var = self.mgr.IthVar(varnum)
                else:
                    self.BDD_to_var_name[self.mgr.IthVar(varnum)] = varname
                    self.var_name_to_BDD[varname] = self.mgr.IthVar(varnum)

                # TODO: check for consecutivity

        # Create a Domain for jx to help with conversion to/from bitvectors
        self.jx_domain = strategy.Domain("_jx", value_mapping=range(self.num_goals), endianness=strategy.Domain.B0_IS_LSB)

    def searchForStates(self, prop_assignments, state_list=None):
        """ Returns an iterator for the subset of all known states (or a subset
            specified in `state_list`) that satisfy `prop_assignments`. """

        if state_list is None:
            state_list_bdd = self.strategy
        else:
            state_list_bdd = self.stateListToBDD(state_list)

        satisfying_state_list_bdd = state_list_bdd & self.propAssignmentToBDD(prop_assignments)

        return self.BDDToStates(satisfying_state_list_bdd)

    def satOne(self, bdd, var_names):
        for vn in var_names:
            test = bdd & ~self.var_name_to_BDD[vn]
            if test:
                bdd = test
            else:
                bdd &= self.var_name_to_BDD[vn]

        return bdd

    def satAll(self, bdd, var_names):
        while bdd:
            one_sat = self.satOne(bdd, var_names)
            yield one_sat
            bdd &= ~one_sat

    def BDDToStates(self, bdd):
        for one_sat in self.satAll(bdd, self.getAllVariableNames() + self.jx_domain.getPropositions()):
            yield self.BDDToState(one_sat)

    def BDDToState(self, bdd):
        prop_assignments = self.BDDToPropAssignment(bdd, self.getAllVariableNames())
        jx = self.getJxFromBDD(bdd)

        return self.states.addNewState(prop_assignments, jx)

    def BDDToPropAssignment(self, bdd, var_names):
        prop_assignments = {k: bool(bdd & self.var_name_to_BDD[k]) for k in var_names}

        return prop_assignments

    def printStrategy(self):
        """ Dump the minterm of the strategy BDD.  For debugging only. """

        self.strategy.PrintMinterm()

    def stateListToBDD(self, state_list, use_next=False):
        return reduce(lambda bdd1, bdd2: bdd1 | bdd2,
                      (self.stateToBDD(s, use_next) for s in state_list))

    def propAssignmentToBDD(self, prop_assignments, use_next=False):
        """ Create a BDD that represents the given *binary* proposition
            assignments (expressed as a dictionary from prop_name[str]->prop_val[bool]).
            If `use_next` is True, all variables will be primed. """

        # Expand all domains in the prop assignments since the BDD operates
        # on binary propositions
        prop_assignments = self.states.expandDomainsInPropAssignment(prop_assignments)

        # Start with the BDD for True
        bdd = self.mgr.ReadOne()

        # Add all the proposition values one by one
        for prop_name, prop_value in prop_assignments.iteritems():
            if use_next:
                prop_name += "'"

            if prop_value:
                bdd &= self.var_name_to_BDD[prop_name]
            else:
                bdd &= ~self.var_name_to_BDD[prop_name]

        return bdd

    def stateToBDD(self, state, use_next=False):
        """ Create a BDD that represents the given state.
            If `use_next` is True, all variables will be primed. """

        state_bdd = self.propAssignmentToBDD(state.getAll(expand_domains=True), use_next)

        if use_next is False:
            # We don't currently use jx in the next
            state_bdd &= self.getBDDFromJx(state.goal_id)

        return state_bdd

    def getAllVariableNames(self, use_next=False):
        if use_next:
            return (v+"'" for v in self.states.getPropositions(expand_domains=True))
        else:
            return self.states.getPropositions(expand_domains=True)

    def getAllVariableBDDs(self, use_next=False):
        return (self.var_name_to_BDD[v] for v in self.getAllVariableNames(use_next))

    def prime(self, bdd):
        # TODO: modify support? error check
        return self._BDDSwapVars(bdd, self.getAllVariableBDDs(use_next=False), self.getAllVariableBDDs(use_next=True))

    def unprime(self, bdd):
        return self._BDDSwapVars(bdd, self.getAllVariableBDDs(use_next=True), self.getAllVariableBDDs(use_next=False))

    def _DDArrayFromList(self, elements):
        # We have to do this silly type conversion because we're using a very loosely-wrapped C library
        dd_array = pycudd.DdArray(len(elements))
        for idx, el in enumerate(elements):
            dd_array[idx] = el

        return dd_array

    def _BDDSwapVars(self, bdd, varset1, varset2):
        # Make sure we have an iterator with a len()
        varset1 = list(varset1)
        varset2 = list(varset2)

        assert len(varset1) == len(varset2)

        dd_varset1 = self._DDArrayFromList(varset1)
        dd_varset2 = self._DDArrayFromList(varset2)

        return bdd.SwapVariables(dd_varset1, dd_varset2, len(varset1))

    def findTransitionableStates(self, prop_assignments, from_state=None):
        """ Return a list of states that can be reached from `from_state`
            and satisfy `prop_assignments`.  If `from_state` is omitted,
            the strategy's current state will be used. """

        if from_state is None:
            from_state = self.current_state

        # If possible, move on to the next goal (only possible if current states fulfils current goal)
        candidates = self._getNextStateBDD(from_state, prop_assignments, "Z")
        if candidates:
            candidate_states = list(self.BDDToStates(candidates))
            for s in candidate_states:
                # add 1 to jx
                s.goal_id = (s.goal_id + 1) % self.num_goals

            return candidate_states

        # If that wasn't possible, try to move closer to the current goal
        candidates = self._getNextStateBDD(from_state, prop_assignments, "Y")
        if candidates:
            return list(self.BDDToStates(candidates))

        # If we've gotten here, something's terribly wrong
        raise RuntimeError("No next state could be found.")

    def _getNextStateBDD(self, from_state, prop_assignments, strat_type):
        # Explanation of the strat_type var (from JTLV code):
        #    0. The strategies that do not change the justice pursued
        #    1. The strategies that change the justice pursued
        if strat_type == "Y":
            strat_type_bdd = ~self.strat_type_var
        elif strat_type == "Z":
            strat_type_bdd = self.strat_type_var
        else:
            raise ValueError("Invalid strategy type")

        next_state_restrictions = self.propAssignmentToBDD(prop_assignments, use_next=True)
        candidates = self.unprime(  self.stateToBDD(from_state)
                                  & self.strategy
                                  & strat_type_bdd
                                  & next_state_restrictions)
        return candidates

    def getBDDFromJx(self, jx):
        return self.propAssignmentToBDD(self.jx_domain.numericValueToPropAssignments(jx))

    def getJxFromBDD(self, bdd):
        return self.jx_domain.propAssignmentsToNumericValue(self.BDDToPropAssignment(bdd, self.jx_domain.getPropositions()))
