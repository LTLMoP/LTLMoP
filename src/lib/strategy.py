#!/usr/bin/env python
# -*- coding: UTF-8 -*-

""" ==============================================================================
    strategy.py - A Strategy object encodes a discrete strategy, which gives a
    system move in response to an environment move (or the reverse, in the case of a counterstrategy).
    ==============================================================================
"""

import math
import re
import logging
import textwrap
import regions
import sys
import collections
import copy
import globalConfig

# TODO: generalize notion of sets of states in a way transparent to both BDD and
# FSA, so we don't end up unnecessarily creating and iterating over state
# objects for things that can be done with a single BDD op
# TODO: should we be using region names instead of objects to avoid
# weird errors if two copies of the same map are used?

def createStrategyFromFile(filename, input_propositions, output_propositions):
    """ High-level method for loading a strategy of any type from file.

        Takes a filename and lists of input and output propositions.
        Returns a fully-loaded instance of a Strategy subclass."""

    # Instantiate the appropriate subclass of strategy
    if filename.endswith(".aut"):
        import fsa
        new_strategy = fsa.FSAStrategy()
    elif filename.endswith(".bdd") or filename.endswith(".add"):
        import bdd
        if filename.endswith(".bdd"):
            new_strategy = bdd.BDDStrategy(add = False)
        else:
            new_strategy = bdd.BDDStrategy(add = True)
    else:
        raise ValueError("Unsupported strategy file type.  Filename must end with either '.aut' or '.bdd'.")

    # Configure and load
    new_strategy.configurePropositions(input_propositions, output_propositions)
    new_strategy.loadFromFile(filename)

    return new_strategy

class Domain(object):
    """ A Domain is a bit-vector abstraction, allowing a proposition to effectively
    have values other than just True and False.

        Domain "x" consists of propositions "x_b0", "x_b1", "x_b2", ..., and the
    value of the domain corresponds to the interpretation of these propositions as
    a binary string (following the order specified by `endianness`).  If
    `value_mapping` is specified, the numeric value of the domain will be used as
    an index into this array, and the corresponding element (must be non-integer)
    will be returned when the proposition value is queried, instead of a number
    (likewise, when setting the value of the proposition, this mapping will be used
    in reverse).

        `num_props` can be used to specify the size of the vector; if not
    specified, this will be automatically calculated based on the size of the
    `value_mapping` array.

    Define a domain:
    >>> animals = ["cat", "dog", "red-backed fairywren", "pseudoscorpion", "midshipman"]
    >>> d = Domain("favorite_animal", animals)

    Notice that by using a domain we've reduced the number of props necessary
    >>> assert d.num_props == 3

    Conversion can go in both directions:
    >>> for value in animals:
    ...     p = d.valueToPropAssignments(value)
    ...     assert value == d.propAssignmentsToValue(p)
    """

    # TODO: add code for generating LTL mutexes
    # TODO: add non-bitvector mode (e.g. region_kitchen)-- but requires mutex!

    # Constants to indicate endianness options
    B0_IS_MSB, B0_IS_LSB = range(2)

    def __init__(self, name, value_mapping=None, endianness=B0_IS_MSB, num_props=None):
        if not re.match(r"^[A-Za-z][A-Za-z0-9_]*$", name) and not name.startswith("_jx"):
            raise ValueError("Name must begin with a letter and contain only alphanumeric characters or underscores.")

        self.name = name

        if endianness not in [Domain.B0_IS_MSB, Domain.B0_IS_LSB]:
            raise ValueError("Invalid endianness: choose either B0_IS_MSB or B0_IS_LSB.")
        self.endianness = endianness

        self.value_mapping = value_mapping
        if num_props is None:
            if value_mapping is None:
                raise TypeError("Cannot create domain without either value_mapping or num_props specified.")
            else:
                # Calculate the minimum number of bits necessary; note that we use max(1,...) because log(1)==0
                self.num_props = max(1, int(math.ceil(math.log(len(value_mapping), 2))))
        else:
            self.num_props = num_props

    def propAssignmentsToValue(self, prop_assignments):
        """ Return the value of this domain, based on a dictionary [prop_name(str)->value(bool)] of
            the values of the propositions composing this domain.
        """
        n = self.propAssignmentsToNumericValue(prop_assignments)

        if self.value_mapping is None:
            return n
        else:
            try:
                return self.value_mapping[n]
            except IndexError:
                relevant_assignments = {k: v for k, v in prop_assignments.iteritems() if k in self.getPropositions()}
                raise ValueError("Invalid assignment of {!r} to domain {!r} ({} > {})".format(relevant_assignments, self.name,
                                                                                              n, len(self.value_mapping)-1))

    def propAssignmentsToNumericValue(self, prop_assignments):
        """ Convert a dictionary [prop_name(str)->value(bool)] of propositions composing this domain
            into an integer value.
        """

        value = 0
        for bit, prop_name in enumerate(self.getPropositions()):
            if prop_name not in prop_assignments:
                raise ValueError("Cannot evaluate domain {!r} because expected subproposition {!r} is undefined.".format(self.name, prop_name))

            if prop_assignments[prop_name]:
                if self.endianness == Domain.B0_IS_MSB:
                    value += 2**((self.num_props-1)-bit)
                else:
                    value += 2**bit

        return value

    def valueToPropAssignments(self, value):
        """ Convert a value into the corresponding dictionary [prop_name(str)->value(bool)]
            of propositions composing this domain
        """

        if self.value_mapping is None:
            if isinstance(value, int):
                n = value
            else:
                raise TypeError("Non-integral values are not permitted without a value_mapping.")
        else:
            n = self.value_mapping.index(value)

        return self.numericValueToPropAssignments(n)

    def numericValueToPropAssignments(self, number):
        """ Convert an integer value into the corresponding dictionary [prop_name(str)->value(bool)]
            of propositions composing this domain
        """

        # Perform input sanity checks
        if not isinstance(number, int):
            raise TypeError("Cannot set domain to non-integral value.")

        if number < 0:
            raise TypeError("Cannot set domain to negative value.")

        # Convert to a left-padded bitstring
        bs = "{0:0>{1}}".format(bin(number)[2:], self.num_props)

        # Create a dictionary based on the bitstring
        return {"{}_b{}".format(self.name, bit):(v=="1") for bit, v in
                enumerate(bs if self.endianness == Domain.B0_IS_MSB else reversed(bs))}

    def getPropositions(self):
        """ Returns a list of the names of the propositions that are covered by this domain. """

        return ["{}_b{}".format(self.name, bit) for bit in range(self.num_props)]

    def __str__(self):
        return '<Domain "{0}" ({0}_b0:{0}_b{1})>'.format(self.name, self.num_props-1)

class State(object):
    """
    A state, at its most basic, consists of a value assignment to propositions
    (represented as a dictionary {proposition name (string) -> proposition value}).

    Additional metadata can be attached as necessary.

    When created, a reference to the parent StateCollection needs to be be
    passed so that the state is aware of its evaluation context.

    For example usage, see the documentation for StateCollection.

    A Note About Multi-Valent Propositions:

        Multivalent propositions (i.e. those whose value can span a Domain), are
    handled fairly flexibly internally, but most of the dynamic translation is
    hidden from the user.

        In general, read access to propositions will always be presented at the
    highest-level possible.  For example, when asking for the value of a state,
    multivalent propositions are presented instead of the underlying binary
    subpropositions (unless explicitly overridden by using the `expand_domains`
    flag provided by some functions).  That said, if one wishes to query the value
    of a subproposition for some reason, its value will be calculated
    automatically.

        In a similar vein, in order to minimize the worries of those using this
    module, multivalent propositions can be written to-- and are stored
    internally-- in one of two ways: either as multiple binary assignments to the
    subpropositions of the domain, or a single value assignment to the domain
    proposition itself.  This latter form is preferred, since it is simplest, and
    internal accounting is biased in this direction.
    """

    def __init__(self, parent, prop_assignments=None):
        """ Create a new state.  Optionally set the state assignment immediately,
            using `prop_assignments`. """

        if not isinstance(parent, StateCollection):
            raise TypeError("The parent of a State must be a StateCollection.")

        self.context = parent
        self.assignment = {}

        # Some optional meta-data
        self.state_id = None  # If you want to give the state a unique identifier
        self.goal_id = None   # Index of currently-pursued goal

        if prop_assignments is not None:
            self.setPropValues(prop_assignments)

    def getName(self):
        if self.state_id is None:
            # Make sure the unique ID is positive
            unique_id = hash(self) % ((sys.maxsize + 1) * 2)
            return "state_{}".format(unique_id)
        else:
            return self.state_id

    def getInputs(self, expand_domains=False):
        """ Return a dictionary of assignments to input propositions for this state.

            If `expand_domains` is True, return only the binary subpropositions
            for domains instead of the usual multivalent proposition. """

        return self.getPropValues(self.context.input_props, expand_domains)

    def getOutputs(self, expand_domains=False):
        """ Return a dictionary of assignments to output propositions for this state.

            If `expand_domains` is True, return only the binary subpropositions
            for domains instead of the usual multivalent proposition. """

        return self.getPropValues(self.context.output_props, expand_domains)

    def getAll(self, expand_domains=False):
        """ Return a dictionary of assignments to all propositions for this state.

            If `expand_domains` is True, return only the binary subpropositions
            for domains instead of the usual multivalent proposition. """

        assignments = self.getInputs(expand_domains)
        assignments.update(self.getOutputs(expand_domains))

        return assignments

    def satisfies(self, prop_assignments):
        """ Returns `True` iff the proposition settings in this state agree with all
            `prop_assignments`.  Any unspecified propositions are treated as don't-cares. """

        return all((self.getPropValue(k) == v for k, v in prop_assignments.iteritems()))

    def getPropValues(self, names, expand_domains=False):
        """ Return a dictionary of assignments to the propositions in `names`
            for this state.

            If `expand_domains` is True, return only the binary subpropositions
            for domains instead of the usual multivalent proposition. """

        prop_values = {p: self.getPropValue(p) for p in names}

        # If expand_domains is True, replace all domain propositions in the
        # return dictionary with their subpropositions
        if expand_domains:
            prop_values = self.context.expandDomainsInPropAssignment(prop_values)

        return prop_values

    def getPropValue(self, name):
        """ Return the value of the proposition `name` in this state.

            (Note: `expand_domains` is not supported here because it would
             entail returning multiple values.  Use getPropValues() for that.) """

        # OK, there are three possibilities for how we will evaluate the name:

        # 1) If this is a normal proposition name, and we know about it,
        #    just return its value directly
        if name in self.assignment:
            return self.assignment[name]

        # 2) If this is the name of a domain for which we only have the
        #    subpropositions, try to upconvert the subpropositions to a single
        #    multivalent proposition
        domain = self.context.getDomainByName(name)
        if domain is not None:
            # Try to calculate the value of the domain
            value = domain.propAssignmentsToValue(self.assignment)

            # Remove the subprops
            for subprop in domain.getPropositions():
                del self.assignment[subprop]

            # Add the multivalent prop
            self.assignment[name] = value

            return value

        # 3) Check to see if this is the name of a subproposition of any domain,
        #    in which case we can calculate the subproposition value
        parent_domain = self.context.getDomainOfProposition(name)
        if parent_domain is not None:
            return parent_domain.valueToPropAssignments(self.getPropValue(parent_domain.name))[name]

        # Otherwise, we'll have to throw an error
        raise ValueError("Proposition of name '{}' is undefined in this state".format(name))

    def setPropValue(self, prop_name, prop_value):
        """ Sets the assignment of propositions `prop_name` to `prop_value` in this state.
            A lot of sanity checking is performed to ensure the name and value are both appropriate. """
        # FIXME: Changing the value of subpropositions after the domain
        # has been upconverted will cause problems

        # Check that this is a known prop_name
        if (prop_name not in self.context.input_props) and \
           (prop_name not in self.context.output_props) and \
           (self.context.getDomainOfProposition(prop_name) is None):
            raise ValueError("Unknown proposition/domain {!r}".format(prop_name))

        # Make sure that the value makes sense
        domain = self.context.getDomainByName(prop_name)
        if domain is None:
            if not isinstance(prop_value, bool):
                raise ValueError("Invalid value of {!r} for proposition {!r}: can only assign boolean values to non-Domain propositions".format(prop_value, prop_name))
        else:
            if prop_value not in domain.value_mapping:
                raise ValueError("Invalid value of {!r} for domain {!r}.  Acceptable values: {!r}".format(prop_value, prop_name, domain.value_mapping))

        # Store the value
        self.assignment[prop_name] = prop_value

    def setPropValues(self, prop_assignments):
        """ Update the assignments in this state according to `prop_assignments`.

            Any existing assignments to propositions not mentioned in `prop_assignments`
            are untouched. """

        for prop_name, prop_value in prop_assignments.iteritems():
            self.setPropValue(prop_name, prop_value)

    def getLTLRepresentation(self, mark_players=True, use_next=False, include_inputs=True, swap_players=False):
        """ Returns an LTL formula representing this state.

            If `mark_players` is True, input propositions are prepended with
            "e.", and output propositions are prepended with "s.". (If `swap_players`
            is True, these labels will be reversed [this feature is used by Mopsy])

            If `use_next` is True, all propositions will be modified by a single
            "next()" operator.  `include_env`, which defaults to True,
            determines whether to include input propositions in addition to
            output propositions. """

        # Make a helpful little closure for adding operators to bare props
        def decorate_prop(prop, polarity):
            #### TEMPORARY HACK: REMOVE ME AFTER OTHER COMPONENTS ARE UPDATED!!!
            # Rewrite proposition names to make the old bitvector system work
            # with the new one
            prop = re.sub(r'^([se]\.)region_b(\d+)$', r'\1bit\2', prop)
            #################################################################

            if use_next:
                prop = "next({})".format(prop)
            if polarity is False:
                prop = "!"+prop
            return prop

        if swap_players:
            env_label, sys_label = "s.", "e."
        else:
            env_label, sys_label = "e.", "s."

        sys_state = " & ".join((decorate_prop(sys_label+p, v) for p, v in \
                                self.getOutputs(expand_domains=True).iteritems()))

        if include_inputs:
            env_state = " & ".join((decorate_prop(env_label+p, v) for p, v in \
                                    self.getInputs(expand_domains=True).iteritems()))
            return " & ".join([env_state, sys_state])
        else:
            return sys_state

    def __eq__(self, other):
        return isinstance(other, State) and hash(self) == hash(other)

    def __hash__(self):
        # We need to call getAll() instead of using self.assignment directly so
        # that the hash is consistent between states with different levels of
        # Domain "up-conversion"
        # TODO: This design choice might be worth reconsidering at some point.
        return hash((id(self.context),
                     frozenset(self.getAll().items()),
                     self.goal_id))

    def __repr__(self):
        return "<State with assignment: inputs = {}, outputs = {} (goal_id = {})>".format(self.getInputs(), self.getOutputs(), self.goal_id)

    def __deepcopy__(self, memo):
        """ Implement a 'medium' copy so that we make a true copy of the assignment dictionary,
            but don't accidentally make copies of proposition values (e.g. region objects). """

        new_state = copy.copy(self)
        new_state.assignment = copy.copy(self.assignment)

        return new_state

class StateCollection(list):
    """
    StateCollection is a simple extension of list, to allow for keeping track of
    meta-information about states, such as which propositions are inputs and which
    are outputs, as well as domains. (These are not class properties of State
    because different StateCollections might have different settings.)

    Create a new state collection:
    >>> states = StateCollection()

    Define some basic true/false propositions:
    >>> states.addInputPropositions(("low_battery",))
    >>> states.addOutputPropositions(("hypothesize", "experiment", "give_up"))

    Define some multi-valent propositions:
    >>> regions = ["kitchen", "living", "bedroom"]
    >>> animals = ["cat", "dog", "red-backed fairywren", "pseudoscorpion", "midshipman"]
    >>> states.addOutputPropositions([Domain("region", regions)])
    >>> states.addInputPropositions([Domain("nearby_animal", animals, Domain.B0_IS_LSB)])

    Create a new state:
    >>> test_assignment = {"region": "bedroom", "nearby_animal": "midshipman",
    ...                    "low_battery": True, "hypothesize": True,
    ...                    "experiment":False, "give_up":False}
    >>> s = states.addNewState(test_assignment)
    >>> assert s.satisfies(test_assignment)

    You can also query and set the state values using low-level subpropositions:
    >>> s2 = states.addNewState(s.getAll(expand_domains=True))
    >>> assert s2.satisfies(test_assignment)

    LTL is available too!
    #>>> s2.getLTLRepresentation()
    #'!e.nearby_animal_b1 & !e.nearby_animal_b0 & e.nearby_animal_b2 & e.low_battery & s.region_b0 & !s.region_b1 & !s.give_up & !s.experiment & s.hypothesize'
    """

    def __init__(self, *args, **kwds):
        self.clearPropositionsAndDomains()
        super(StateCollection, self).__init__(*args, **kwds)

    def clearPropositionsAndDomains(self):
        """ Remove all propositions and domain definitions. """

        self.input_props = []
        self.output_props = []
        self.domains = []

    def clearStates(self):
        """ Remove all states. """

        del self[:]

    def _addPropositions(self, prop_list, target):
        """ Take a mixed list of plain propositions (str) and Domain objects
            and load them into the proper `target` location. """

        # Make sure we don't accept bare strings, which are also iterables
        if not isinstance(prop_list, (tuple, list)):
            raise TypeError("prop_list must be a list or tuple")

        for prop in prop_list:
            if isinstance(prop, Domain):
                self.domains.append(prop)
                target.append(prop.name)
            else:
                target.append(prop)

    def addInputPropositions(self, prop_list):
        """ Register the propositions in `prop_list` as input propositions.
            Each element of `prop_list` may be either a bare string,
            which is treated as the name of a binary proposition,
            or a Domain object, which indicates a multivalent proposition. """

        self._addPropositions(prop_list, self.input_props)

    def addOutputPropositions(self, prop_list):
        """ Register the propositions in `prop_list` as output propositions.
            Each element of `prop_list` may be either a bare string,
            which is treated as the name of a binary proposition,
            or a Domain object, which indicates a multivalent proposition. """

        self._addPropositions(prop_list, self.output_props)

    def expandDomainsInPropAssignment(self, prop_assignments):
        """ Replace all domain propositions in the dictionary with their subpropositions """

        prop_values = copy.copy(prop_assignments)

        for n in prop_assignments.keys():
            domain = self.getDomainByName(n)
            if domain is None:
                continue
            prop_values.update(domain.valueToPropAssignments(prop_values[n]))
            del prop_values[n]

        return prop_values

    def getPropositions(self, expand_domains=False):
        """ Return a list of all known proposition names. """

        # TODO: this is kind of redundant with some code in State.getPropValues()
        prop_list = self.input_props + self.output_props

        # If expand_domains is True, replace all domain propositions in the
        # return list with their subpropositions
        if expand_domains:
            for d in self.domains:
                prop_list.remove(d.name)
                prop_list.extend(d.getPropositions())

        return prop_list

    def addNewState(self, prop_assignments=None, goal_id=None):
        """ Create a new state with the assignment `prop_assignment` and
            goal ID `goal_id` and add it to the StateCollection.

            Returns the new state. """

        new_state = State(self, prop_assignments)
        new_state.goal_id = goal_id
        self.append(new_state)

        return new_state

    def getDomainOfProposition(self, prop_name):
        """ Returns the Domain object for which proposition `prop_name`
            is a subproposition.

            If no such Domain is found, returns None. """

        # TODO: We could do this faster by just using the beginning of the
        #       proposition's name-- it would be more hard-coded, though.
        return next((d for d in self.domains if prop_name in d.getPropositions()), None)

    def getDomainByName(self, name):
        """ Returns the Domain object with name `name`.

            If no such Domain is found, returns None. """

        return next((d for d in self.domains if d.name == name), None)

class Strategy(object):
    """
    A Strategy object encodes a discrete strategy, which gives a
    system move in response to an environment move (or the reverse, in the case of
    a counterstrategy).

    Only subclasses of Strategy should be used.
    """

    def __init__(self):
        self.current_state = None

    def configurePropositions(self, input_propositions, output_propositions):
        """ Set the input and output propositions for this strategy.

            `input_propositions` and `output_propositions` must both be lists,
            consisting of any combination of strings (i.e. binary proposition names) and
            strategy.Domain objects (for multivalent propositions).

            All existing definitions will be cleared.

            This must be done before creating any states. """

        self.states.clearPropositionsAndDomains()
        self.states.addInputPropositions(input_propositions)
        self.states.addOutputPropositions(output_propositions)

    def iterateOverStates(self):
        """ Returns an iterator over all known states. """

        return self.searchForStates({})

    def loadFromFile(self, filename):
        """ Load a strategy from a file. """

        logging.info("Loading strategy from file '{}'...".format(filename))

        tic = globalConfig.best_timer()
        self._loadFromFile(filename)
        toc = globalConfig.best_timer()

        logging.info("Loaded in {} seconds.".format(toc-tic))

    def _loadFromFile(self, filename):
        """ The inner function that actually performs file loading
            for loadFromFile(). """

        raise NotImplementedError("Use a subclass of Strategy")

    def searchForStates(self, prop_assignments, state_list=None):
        """ Returns an iterator for the subset of all known states (or a subset
            specified in `state_list`) that satisfy `prop_assignments`. """

        raise NotImplementedError("Use a subclass of Strategy")

    def findTransitionableStates(self, prop_assignments, from_state=None):
        """ Return a list of states that can be reached from `from_state`
            and satisfy `prop_assignments`.  If `from_state` is omitted,
            the strategy's current state will be used. """

        raise NotImplementedError("Use a subclass of Strategy")

    def searchForOneState(self, prop_assignments, state_list=None):
        """ Iterate through all known states (or a subset specified in `state_list`)
            and return the first one that matches `prop_assignments`.

            Returns None if no such state is found.  """

        return next(self.searchForStates(prop_assignments, state_list), None)

    def exportAsDotFile(self, filename, regionMapping, starting_states=None):
        """ Output an explicit-state strategy to a .dot file of name `filename`.
            (For use with GraphViz.) """

        if starting_states is None:
            starting_states = self.iterateOverStates()

        processed_states = set()
        states_to_process = collections.deque(starting_states)

        # We will save the transitions in memory to output at the end of the file, after the states
        transitions = []

        with open(filename, 'w') as f_out:
            # Header
            f_out.write(textwrap.dedent("""
                digraph A {
                    rankdir = LR;
                    overlap = false;
                    ratio = "compress";
            """))

            # Define a helper function
            def pprint_assignment(name, val):
                if isinstance(val, bool):
                    return name if val else "!"+name
                elif isinstance(val, regions.Region):
                    # annotate any pXXX region names with their human-friendly name
                    for rname, subregs in regionMapping.iteritems():
                        if val.name in subregs:
                            break
                    val = val.name + " ("+ rname +")" #parent region
                return "{} = {}".format(name, val)

            while states_to_process:
                this_state = states_to_process.pop()

                # Skip this if we processed this already earlier on the stack
                if this_state in processed_states:
                    continue

                state_label = "\\n".join((pprint_assignment(k, v)
                                          for k, v in this_state.getOutputs().iteritems()))
                state_label += "\\n[Goal #{}]".format(this_state.goal_id)
                f_out.write('\t{} [style="bold", width=0, height=0, fontsize=20, label="{}"];\n'\
                            .format(this_state.getName(), state_label))

                processed_states.add(this_state)

                for next_state in self.findTransitionableStates({}, from_state=this_state):
                    trans_label = "\\n".join((pprint_assignment(k, v)
                                              for k, v in next_state.getInputs().iteritems()))
                    transitions.append('\t{} -> {} [style="bold", arrowsize=1.5, fontsize=20, label="{}"];\n'\
                                .format(this_state.getName(), next_state.getName(), trans_label))

                    if next_state not in processed_states:
                        states_to_process.append(next_state)

            f_out.writelines(transitions)

            # Close the digraph
            f_out.write("} \n")

def TestLoadAndDump(spec_filename):
    import project
    import pprint
    import strategy  # When running from the context of __name__ == "__main__", we need to
                     # explicitly import like this or else things get weird... e.g. __main__.Domain
                     # and strategy.Domain become different things

    ### Load the project
    proj = project.Project()
    proj.loadProject(spec_filename)
    spec_map = proj.loadRegionFile(decomposed=True)

    ### Load the strategy
    region_domain = strategy.Domain("region", spec_map.regions, Domain.B0_IS_MSB)

    strat = createStrategyFromFile(proj.getStrategyFilename(),
                                   proj.enabled_sensors,
                                   proj.enabled_actuators + proj.all_customs + [region_domain])

    ### Choose a starting state
    initial_region = spec_map.regions[0]  # Hopefully this is a valid region to start from...
    start_state = strat.searchForOneState({"region": initial_region, "radio": False})
    #pprint.pprint(strat.findTransitionableStates({}, from_state=start_state))

    ### Dump the strategy from this initial state
    strat.exportAsDotFile("strategy_test.dot", starting_states = [start_state])

if __name__ == "__main__":
    logging.info("Running doctests...")
    import doctest
    doctest.testmod()

    if len(sys.argv) > 1:
        logging.info("Running file load/dump test for {!r}...".format(sys.argv[1]))
        TestLoadAndDump(sys.argv[1])
