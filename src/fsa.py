#!/usr/bin/env python

""" ======================================
    fsa.py - Finite-State Automaton module
    ======================================
    
    Defines a specific model for finite state automata, including a method to read in files
    produced by JTLV and a method to execute the automaton.
"""

import math, re, sys, random, os, subprocess, time
from regions import *
import numpy
import fileMethods


###########################################################

class FSA_State:
    """
    Each state in the automaton is an object.

    WARNING/FIXME: Since all states belong to a list within the Automaton object, the states may
    also be referred to just by their index within that list.  This can get confusing sometimes.
    """
    def __init__ (self, name, inputs, outputs, transitions):
        self.name = name                    # The name of the state (currently the number assigned by TLV)

        # NOTE: All input/output values are STRINGS.  Please cast and compare appropriately.

        self.inputs = inputs                # A dict of each sensor name to the value it
                                            # must have to transition to this state
        self.outputs = outputs              # A dict of each output proposition name to the
                                            # value it has in this state
                                            # WARNING: This list includes "bitX" propositions from
                                            # region encoding.  You may wish to ignore these.
        self.transitions = transitions      # A list of state objects that may be transitioned to
                                            # from this state

###########################################################

class Automaton:
    """
    An automaton object is a collection of state objects along with information about the
    current state of the automaton when being executed. 
    """

    def __init__ (self, regions, sensor_handler, actuator_handler, motion_handler):
        """
        Creates a new automaton.

        You need to pass a list of region objects (for mapping between region names and numbers), 
        and handler objects for sensors, actuators, and region-to-region movement.
        """

        self.states = []    # A collection of state objects belonging to the automaton

        self.regions = regions
        self.num_bits = int(numpy.ceil(numpy.log2(len(regions))))  # Number of bits necessary to encode all regions

        # Store references to the handlers
        self.sensor_handler = sensor_handler
        self.actuator_handler = actuator_handler
        self.motion_handler = motion_handler

        # Variables for keeping track of the current state
        self.current_state = None
        self.current_region = None
        self.current_outputs = {}

    def stateWithName(self, name):
        """
        Find the state with the given name 
        """
        for i in range(len(self.states)):
            if(self.states[i].name == name):
                return self.states[i] 

        print "ERROR: Can't find state with name %s!" % (name)
        return None

    def dumpStates(self):
        """
        Print out the contents of the automaton in a human-readable format
        """
        for state in self.states:
            print "Name: ", state.name
            print "Inputs: "
            for key, val in state.inputs.iteritems():
                print key + " = " + val
            print "Outputs: "
            for key, val in state.outputs.iteritems():
                print key + " = " + val
            print "Transitions: "
            for trans in state.transitions:
                print trans.name

    def updateOutputs(self, state):
        """
        Update the values of current outputs in our execution environment to reflect the output
        proposition values associated with the given state
        """

        for key, output_val in state.outputs.iteritems():
            # Skip any "bitX" region encodings
            if re.match('^bit\d+$', key): continue 

            new_val = (output_val == "1")
            
            if new_val != self.current_outputs[key]:
                # The state of this output proposition has changed!

                print "Output proposition \"%s\" is now %s!" % (key, str(new_val))
                
                # Run any actuator handlers if appropriate
                if key in self.actuators:
                    self.motion_handler.gotoRegion(self.current_region, self.current_region)  # Stop, in case actuation takes time
                    self.actuator_handler.setActuator(key, new_val)

                self.current_outputs[key] = new_val

    def regionFromState(self, state):
        """
        Given a state object, look at its 'bitX' outputs to determine the region encoded,
        and return the NUMBER of this region.
        """
        region = 0
        for bit in range(self.num_bits):
            if (int(state.outputs["bit" + str(bit)]) == 1):
                # bit0 is MSB
                region += int(2**(self.num_bits-bit-1))

        return region

    def loadFile(self, filename, sensors, actuators, custom_props):
        """
        Create an automaton by reading in a file produced by TLV.

        In addition to a filename, you also need to provide a list of sensor names (so that we can tell the difference between system and environment propositions when reading the file), and a list of actuator names (so we can distinguish internal state propositions from outputs).

        Basically just a lot of regexes.
        """

        # These will be used later by updateOutputs() and findTransitionableState()
        self.actuators = actuators
        self.sensors = sensors
        self.custom_props = custom_props

        FILE = open(filename,"r")
        fsa_description = FILE.read()
        FILE.close()

        ###################
        # Read in states: #
        ###################

        # A magical regex to slurp up a state and its information all at once
        p = re.compile(r"State (?P<num>\d+) with rank (?P<rank>[\d-]+) -> <(?P<conds>(?:\w+:\d(?:, )?)+)>", re.IGNORECASE|re.MULTILINE)
        m = p.finditer(fsa_description)

        # Now, for each state we find:
        for match in m:

            # Get the number (at least the number that TLV assigned the state; TLV deletes states
            # during optimization, resulting in non-consecutive numbering which would be bad for binary
            # encoding efficiency, so we don't use these numbers internally except as state names) 
            # and rank (an irrelevant synthesis byproduct that we only read in for completeness). 
            # This is the easy part.

            number = match.group('num')
            rank = match.group('rank')

            # A regex so we can iterate over "PROP = VALUE" terms
            p2 = re.compile(r"(?P<var>\w+):(?P<val>\d)", re.IGNORECASE|re.MULTILINE)
            m2 = p2.finditer(match.group('conds'))

            inputs = {}
            outputs = {} 

            # So, for each of these terms:
            for new_condition in m2:
                var = new_condition.group('var')
                val = new_condition.group('val') 

                # Ignore internal "current goal" propositions
                if var.startswith('s_'): continue
                
                # And then put it in the right place!

                if var not in sensors:
                    # If it's not a sensor proposition, then it's an output proposition
                    outputs[var]=val
                else:
                    # Oh hey it's a sensor
                    inputs[var]=val

            # We'll add transitions later; first we have to create all the states so we can
            # refer to them when we define transitions
            transitions = []

            # Create the state and add it to our collection
            newstate = FSA_State(number, inputs, outputs, transitions)
            newstate.rank=rank
            self.states.append(newstate)


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

            # Change the references to state names into references to the corresponding state objects
            ends = map(lambda name: self.stateWithName(name), ends)

            # Stick these transitions onto the appropriate state
            self.stateWithName(start).transitions = ends

        # All done, hooray!
        print "Loaded %d states." % len(self.states)
        #self.dumpStates()

    def writeDot(self, filename):
        """
        Write a dot file so we can look at the automaton visually.
        """
        
        FILE = open(filename,"w")

        # Write the header
        FILE.write('digraph A { \n')
        FILE.write('\trankdir=TB;\n')
        #FILE.write('\tratio = 0.75;\n')
        FILE.write('\tsize = "8.5,11";\n')
        FILE.write('\toverlap = false;\n')
        FILE.write('\tlayout = hierarchical;\n')

        # Write the states with region and outputs that are true
        for state in self.states:
            FILE.write('\ts'+ state.name + ' [style=\"bold\",width=0,height=0, fontsize = 20, label=\"')
            stateRegion = self.regionFromState(state)
            FILE.write( self.regions[stateRegion].name + ' \\n ')
            for key in state.outputs.keys():
                if state.outputs[key] == '1' and not re.match('^bit\d+$',key):
                    # Only propositions that are TRUE and not bitXs are written in the state
                    FILE.write( key + ' \\n ')
            FILE.write( "("+state.rank + ')\\n ')
            FILE.write('\" ];\n')

        # Write the transitions with the input labels (only inputs that are true)
        for state in self.states:
            for nextState in state.transitions:
                FILE.write('\ts'+ state.name +' -> s'+ nextState.name +'[style=\"bold\", arrowsize = 1, fontsize = 20, label=\"')
                # Check the next state to figure out which inputs have to be on
                for key in nextState.inputs.keys():
                    if nextState.inputs[key] == '1':
                        FILE.write( key + ' \\n ')
                FILE.write('\" ];\n')    
        
        FILE.write('} \n')
        FILE.close()

    def findTransitionableStates(self, initial=False):
        """
        Returns a list of states that we could conceivably transition to, given
        the environment state (determined by querying the sensor handler)

        If ``initial`` is true, the current region and output propositions will constrain
        state selection as well. 
        """

        candidates = []

        # Define our pool of states to select from
        if initial:
            state_list = self.states
        else:
            state_list = self.current_state.transitions + [self.current_state] # Allow for self-transition

        # Take a snapshot of our current sensor readings
        # This is so we don't risk the readings changing in the middle of our state search
        sensor_state = {}
        for sensor in self.sensors:
            sensor_state[sensor] = self.sensor_handler.getSensorValue(sensor) 

        for state in state_list:
            okay = True

            if initial:
                # First see if we can be in the state given our current region
                if self.regionFromState(state) != self.current_region: continue

                # Now check whether our current output values match those of the state
                for key, value in state.outputs.iteritems():
                    # Ignore "bitX" output propositions
                    if re.match('^bit\d+$', key): continue

                    if self.current_outputs[key] != (value == "1"):
                        okay = False
                        break

                if not okay: continue

            # Now check whether our current sensor values match those of the state
            for key, value in state.inputs.iteritems(): 
                if sensor_state[key] != (value == "1"):                    
                    okay = False
                    break

            if okay:
                candidates.append(state)

        return candidates

    def chooseInitialState(self, init_region, init_outputs):
        """
        Search through all our states to find one that satisfies our current system and environment states,
        so that we may begin our execution from there.
        
        * ``init_region`` is the number of our starting region
        * ``init_outputs`` is a list of output proposition names that are TRUE initially.
        """

        self.current_region = init_region

        for output in (self.actuators + self.custom_props):
            self.current_outputs[output] = (output in init_outputs)

        candidates = self.findTransitionableStates(initial=True)

        if len(candidates) == 0: # Uh oh; that's no good
            print "(FSA) OH NO, where do I start?! (No suitable initial state found)"
            return None

        # If there's more than one candidate, let's go for variety
        self.current_state = random.choice(candidates)

        # These variables need to be cleared at the beginning of each run
        self.last_next_states = []
        self.next_state = None
        self.next_region = None

        # Bring our actuator states up-to-date
        for key, output_val in self.current_state.outputs.iteritems():
            # Skip any "bitX" region encodings
            if re.match('^bit\d+$', key): continue 
            self.actuator_handler.setActuator(key, output_val)

        return self.current_state

    def runIteration(self):
        """
        Run, run, run the automaton!  (For one evaluation step)
        """

        # Let's try to transition
        next_states = self.findTransitionableStates()

        # Make sure we have somewhere to go
        if len(next_states) == 0:
            # Well darn!
            print "(FSA) ERROR: Could not find a suitable state to transition to!"
            return

        # Only allow self-transitions if that is the only option!
        if len(next_states) > 1 and self.current_state in next_states:
            next_states.remove(self.current_state)
        
        # See if we're beginning a new transition
        if next_states != self.last_next_states:
            # NOTE: The last_next_states comparison is also to make sure we don't
            # choose a different random next-state each time, in the case of multiple choices
            self.next_state = random.choice(next_states)	
            self.next_region = self.regionFromState(self.next_state)
            self.last_next_states = next_states

            # See what we, as the system, need to do to get to this new state
            if self.next_region != self.current_region:
                ### We're going to a new region
                print "Heading to region %s..." % self.regions[self.next_region].name
                # In this case, we can't move into the next state until we've physically reached the new region
            else:
                ### The state changed, but the region didn't
                self.current_state = self.next_state  # We can transition immediately

            # Actuate anything that might be necessary
            self.updateOutputs(self.next_state)

        # Move one step towards the next region (or stay in the same region)
        # TODO: Use the "last" controllers?
        arrived = self.motion_handler.gotoRegion(self.current_region, self.next_region)

        if arrived:
            ### The move handler has told us that we have finally reached our destination region
            # TODO: Finish this check to see whether actually inside next region that we expected:
            #    # Check what region we're in.
            #    pt = rev_coordmap(wx.Point(px,py))
            #    for i, region in enumerate(rfi.regions):
            #        if region.objectContainsPoint(*pt):
            #            return i
            #    return -1

            print "Crossed border from %s to %s!" % (self.regions[self.current_region].name, self.regions[self.next_region].name)
            self.current_state = self.next_state   
            self.current_region = self.next_region
