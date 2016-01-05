#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import re
import strategy
import logging
import sys
import time
from collections import defaultdict

class FSAStrategy(strategy.Strategy):
    """
    An automaton object is a collection of state objects along with information about the
    current state of the automaton when being executed.
    """

    def __init__(self):
        super(FSAStrategy, self).__init__()

        # A collection of state objects belonging to the automaton
        self.states = strategy.StateCollection()

        # A data structure for recording valid transitions between states
        self.transitions = defaultdict(lambda: defaultdict(bool)) # (state1, state2) -> T/F

    def _loadFromFile(self, filename):
        """
        Create an automaton by reading in a file produced by a synthesizer,
        such as JTLV or Slugs.

        Basically just a lot of regexes.
        """

        # Clear any existing states
        self.states.clearStates()

        # Initialize our state ID -> state object mapping, for fast lookup
        state_by_id = {}

        # Load the whole file into memory
        with open(filename, "r") as f:
            fsa_description = f.read()

        ###################
        # Read in states: #
        ###################

        # A magical regex to slurp up a state and its information all at once
        p = re.compile(r"State (?P<state_id>\d+) with rank (?P<goal_id>[\d\(\),-]+) -> <(?P<conds>(?:\w+:\d(?:, )?)+)>", re.IGNORECASE|re.MULTILINE)

        m = p.finditer(fsa_description)

        # Now, for each state we find:
        for match in m:
            # We'll add transitions later; first we have to create all the
            # states so we can refer to them when we define transitions

            # Get the State ID (at least the number that JTLV assigned the
            # state; JTLV deletes states during optimization, resulting in
            # non-consecutive numbering which would be bad for binary encoding
            # efficiency, so we don't use these numbers internally except as
            # state names) and Goal ID ("rank" is a misnomer in the JTLV
            # output; actually corresponds to index of currently pursued goal
            # -- aka "jx").  This is the easy part.

            new_state = self.states.addNewState()
            new_state.state_id = match.group('state_id')
            new_state.goal_id = match.group('goal_id')

            # Counterstrategies may have states without all proposition values
            # defined, so we will leave them as None's.
            # TODO: This weakens error-checking. Maybe make this only apply
            # if specifically asked for when dealing with counter-strategies?
            for prop_name in self.states.getPropositions(expand_domains=True):
                new_state.setPropValue(prop_name, None)

            # A regex so we can iterate over "PROP = VALUE" terms
            p2 = re.compile(r"(?P<var>\w+):(?P<val>\d)", re.IGNORECASE|re.MULTILINE)
            for prop_setting in p2.finditer(match.group('conds')):
                # Set the value of the proposition, casting string "0" or "1" to
                # appropriate boolean values
                prop_name, prop_value = prop_setting.groups()

                #### TEMPORARY HACK: REMOVE ME AFTER OTHER COMPONENTS ARE UPDATED!!!
                # Rewrite proposition names to make the old bitvector system work
                # with the new one
                prop_name = re.sub(r'^bit(\d+)$', r'region_b\1', prop_name)
                #################################################################

                if prop_value == "0":
                    new_state.setPropValue(prop_name, False)
                elif prop_value == "1":
                    new_state.setPropValue(prop_name, True)
                else:
                    raise ValueError("Proposition '{}' value of {!r} in state {} is invalid.".format(prop_name, prop_value, new_state.state_id))

            # Update mapping
            state_by_id[new_state.state_id] = new_state

        ########################
        # Read in transitions: #
        ########################

        # Another simple regex, this time for reading in transition definitions
        p = re.compile(r"State (?P<start>\d+)[^\n]+\s*With successors : (?P<ends>(?:\d+(?:, )?)+)", re.IGNORECASE|re.MULTILINE)
        m = p.finditer(fsa_description)

        # For each line:
        for match in m:
            # Get the state that the transitions come FROM
            start = match.group('start')
            # And make a list of the states that the transitions go TO
            ends = match.group('ends').split(', ')

            for end in ends:
                self.transitions[state_by_id[start]][state_by_id[end]] = True

        # All done, hooray!
        logging.info("Loaded %d states.", len(self.states))

    def searchForStates(self, prop_assignments, state_list=None):
        """ Returns an iterator for the subset of all known states (or a subset
            specified in `state_list`) that satisfy `prop_assignments`. """

        if state_list is None:
            state_list = self.states

        satisfying_states = (s for s in state_list if s.satisfies(prop_assignments))

        return satisfying_states

    def findTransitionableStates(self, prop_assignments, from_state=None):
        """ Return a list of states that can be reached from `from_state`
            and satisfy `prop_assignments`.  If `from_state` is omitted,
            the strategy's current state will be used. """

        if from_state is None:
            if self.current_state is None:
                raise ValueError("You must specify from_state if no current_state is set.")
            from_state = self.current_state

        transitionable_states = self.searchForStates(prop_assignments, state_list=self.transitions[from_state])

        return list(transitionable_states)
