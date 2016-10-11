#!/usr/bin/env python

""" =================================================
    execute.py - Top-level hybrid controller executor
    =================================================

    This module executes a hybrid controller for a robot in a simulated or real environment.

    :Usage: ``execute.py [-hn] [-p listen_port] [-a automaton_file] [-s spec_file]``

    * The controlling automaton is imported from the specified ``automaton_file``.

    * The supporting handler modules (e.g. sensor, actuator, motion control, simulation environment initialization, etc)
      are loaded according to the settings in the config file specified as current in the ``spec_file``.

    * If no port to listen on is specified, an open one will be chosen randomly.
    * Unless otherwise specified with the ``-n`` or ``--no_gui`` option, a status/control window
      will also be opened for informational purposes.
"""

import sys, os, getopt, textwrap
import threading, subprocess, time


# Climb the tree to find out where we are
p = os.path.abspath(__file__)
t = ""
while t != "src":
    (p, t) = os.path.split(p)
    if p == "":
        print "I have no idea where I am; this is ridiculous"
        sys.exit(1)

sys.path.append(os.path.join(p,"src","lib"))

import fsa, project
import handlerSubsystem
import strategy
from copy import deepcopy
from SimpleXMLRPCServer import SimpleXMLRPCServer, SimpleXMLRPCRequestHandler
import xmlrpclib
import socket
import random
import math
import traceback
from resynthesis import ExecutorResynthesisExtensions
from executeStrategy import ExecutorStrategyExtensions
import globalConfig, logging


####################
# HELPER FUNCTIONS #
####################

def usage(script_name):
    """ Print command-line usage information. """

    print textwrap.dedent("""\
                              Usage: %s [-hn] [-p listen_port] [-a automaton_file] [-s spec_file]

                              -h, --help:
                                  Display this message
                              -n, --no-gui:
                                  Do not show status/control window
                              -p PORT, --xmlrpc-listen-port PORT:
                                  Listen on PORT for XML-RPC calls
                              -a FILE, --aut-file FILE:
                                  Load automaton from FILE
                              -s FILE, --spec-file FILE:
                                  Load experiment configuration from FILE """ % script_name)

class LTLMoPExecutor(ExecutorStrategyExtensions,ExecutorResynthesisExtensions, object):
    """
    This is the main execution object, which combines the synthesized discrete automaton
    with a set of handlers (as specified in a .config file) to create and run a hybrid controller
    """

    def __init__(self):
        """
        Create a new execution context object
        """
        super(LTLMoPExecutor, self).__init__()

        self.proj = project.Project() # this is the project that we are currently using to execute
        self.strategy = None

        # Choose a timer func with maximum accuracy for given platform
        if sys.platform in ['win32', 'cygwin']:
            self.timer_func = time.clock
        else:
            self.timer_func = time.time

        self.externalEventTarget = None
        self.externalEventTargetRegistered = threading.Event()
        self.postEventLock = threading.Lock()
        self.runStrategy = threading.Event()  # Start out paused
        self.alive = threading.Event()
        self.alive.set()

        self.current_outputs = {}     # keep track on current outputs values (for actuations)

    def postEvent(self, eventType, eventData=None):
        """ Send a notice that an event occurred, if anyone wants it """

        with self.postEventLock:
            if self.externalEventTarget is None:
                return

            try:
                self.externalEventTarget.handleEvent(eventType, eventData)
            except socket.error as e:
                logging.warning("Could not send event to remote event target: %s", e)
                logging.warning("Forcefully unsubscribing target.")
                self.externalEventTarget = None

    def loadSpecFile(self, filename):
        # Update with this new project
        self.proj = project.Project()
        self.proj.loadProject(filename)
        self.hsub = handlerSubsystem.HandlerSubsystem(self, self.proj.project_root)


        # Tell GUI to load the spec file
        self.postEvent("SPEC", self.proj.getFilenamePrefix() + ".spec")

    def loadAutFile(self, filename):
        """
        This function loads the the .aut/.bdd file named filename and returns the strategy object.
        filename (string): name of the file with path included
        """
        region_domain = strategy.Domain("region",  self.proj.rfi.regions, strategy.Domain.B0_IS_MSB)
        strat = strategy.createStrategyFromFile(filename,
                                                self.proj.enabled_sensors,
                                                self.proj.enabled_actuators + self.proj.all_customs +  [region_domain])

        return strat

    def _getCurrentRegionFromPose(self, rfi=None):
        # TODO: move this to regions.py
        if rfi is None:
            rfi = self.proj.rfi

        pose = self.hsub.coordmap_lab2map(self.hsub.getPose())

        region = next((i for i, r in enumerate(rfi.regions) if r.name.lower() != "boundary" and \
                        r.objectContainsPoint(*pose)), None)

        if region is None:
            logging.warning("Pose of {} not inside any region!".format(pose))

        return region

    def shutdown(self):
        self.runStrategy.clear()
        logging.info("QUITTING.")

        for h in self.hsub.handler_instance:
            if hasattr(h, "_stop"):
                logging.debug("Calling _stop() on {}".format(h.__class__.__name__))
                h._stop()
            else:
                logging.debug("{} does not have _stop() function".format(h.__class__.__name__))

        self.alive.clear()

    def pause(self):
        """ pause execution of the automaton """
        self.runStrategy.clear()
        time.sleep(0.1) # Wait for FSA to stop
        self.postEvent("PAUSE")

    def resume(self):
        """ start/resume execution of the automaton """
        self.runStrategy.set()

    def isRunning(self):
        """ return whether the automaton is currently executing """
        return self.runStrategy.isSet()

    def registerExternalEventTarget(self, address):
        self.externalEventTarget = xmlrpclib.ServerProxy(address, allow_none=True)

        # Redirect all output to the log
        redir = RedirectText(self.externalEventTarget.handleEvent)

        sys.stdout = redir
        sys.stderr = redir

        self.externalEventTargetRegistered.set()

    def initialize(self, spec_file, strategy_file, firstRun=True):
        """
        Prepare for execution, by loading and initializing all the relevant files (specification, map, handlers, strategy)
        If `firstRun` is true, all handlers will be imported; otherwise, only the motion control handler will be reloaded.
        """

        # load project only first time; otherwise self.proj is modified in-place
        # TODO: make this less hacky
        if firstRun:
            self.loadSpecFile(spec_file)

            if self.proj.compile_options['decompose']:
                self.proj.rfiold = self.proj.rfi  # Save the undecomposed regions

        if self.proj.compile_options['decompose']:
            self.proj.rfi = self.proj.loadRegionFile(decomposed=True)

        if self.proj.current_config == "":
            logging.error("Can not simulate without a simulation configuration.")
            logging.error("Please create one by going to [Run] > [Configure Simulation...] in SpecEditor and then try again.")
            sys.exit(2)

        logging.info("Setting current executing config...")
        self.hsub.setExecutingConfig(self.proj.current_config)

        # make sure the coord transformation function is ready
        # get the main robot config
        robot_config = self.hsub.executing_config.getRobotByName(self.hsub.executing_config.main_robot)
        self.hsub.coordmap_map2lab, self.hsub.coordmap_lab2map = robot_config.getCoordMaps()
        self.proj.coordmap_map2lab, self.proj.coordmap_lab2map = robot_config.getCoordMaps()


        # Import the relevant handlers
        if firstRun:
            # Instantiate all handlers
            logging.info("Instantiate all handlers...")
            self.hsub.instantiateAllHandlers()

            logging.info("Preparing proposition mapping...")
            self.hsub.prepareMapping()
        else:
            #print "Reloading motion control handler..."
            #self.proj.importHandlers(['motionControl'])
            pass

        # We are done initializing at this point if there is no aut file yet
        if strategy_file is None:
            return

        # TODO: maybe an option for BDD here later
        # Load automaton file
        new_strategy = self.loadAutFile(strategy_file)

        if firstRun:
            ### Wait for the initial start command
            logging.info("Ready.  Press [Start] to begin...")
            self.runStrategy.wait()

        ### Figure out where we should start from by passing proposition assignments to strategy and search for initial state
        ### pass in sensor values, current actuator and custom proposition values, and current region object

        ## Region
        # FIXME: make getcurrentregion return object instead of number, also fix the isNone check
        init_region = self.proj.rfi.regions[self._getCurrentRegionFromPose()]
        if init_region is None:
            logging.error("Initial pose not inside any region!")
            sys.exit(-1)

        logging.info("Starting from initial region: " + init_region.name)
        init_prop_assignments = {"region": init_region}

        # initialize all sensor and actuator methods
        logging.info("Initializing sensor and actuator methods...")
        self.hsub.initializeAllMethods()


        ## outputs
        if firstRun or self.strategy is None:
            # save the initial values of the actuators and the custom propositions
            for prop in self.proj.enabled_actuators + self.proj.all_customs:
                self.current_outputs[prop] = (prop in self.hsub.executing_config.initial_truths)

        init_prop_assignments.update(self.current_outputs)

        ## inputs
        init_prop_assignments.update(self.hsub.getSensorValue(self.proj.enabled_sensors))

        #search for initial state in the strategy
        init_state = new_strategy.searchForOneState(init_prop_assignments)

        if init_state is None:
            logging.error("No suitable initial state found; unable to execute. Quitting...")
            sys.exit(-1)
        else:
            logging.info("Starting from state %s." % init_state.state_id)

        self.strategy = new_strategy
        self.strategy.current_state = init_state

    def run(self):
        ### Get everything moving
        # Rate limiting is approximately 20Hz
        avg_freq = 20
        last_gui_update_time = 0

        # FIXME: don't crash if no spec file is loaded initially
        while self.alive.isSet():
            # Idle if we're not running
            if not self.runStrategy.isSet():
                self.hsub.setVelocity(0,0)

                # wait for either the FSA to unpause or for termination
                while (not self.runStrategy.wait(0.1)) and self.alive.isSet():
                    pass

            # Exit immediately if we're quitting
            if not self.alive.isSet():
                break

            self.prev_outputs = self.strategy.current_state.getOutputs()
            self.prev_z = self.strategy.current_state.goal_id

            tic = self.timer_func()
            self.runStrategyIteration()
            toc = self.timer_func()

            #self.checkForInternalFlags()

            # Rate limiting of execution and GUI update
            while (toc - tic) < 0.05:
                time.sleep(0.005)
                toc = self.timer_func()

            # Update GUI
            # If rate limiting is disabled in the future add in rate limiting here for the GUI:
            # if show_gui and (timer_func() - last_gui_update_time > 0.05)
            avg_freq = 0.9 * avg_freq + 0.1 * 1 / (toc - tic) # IIR filter
            self.postEvent("FREQ", int(math.ceil(avg_freq)))
            pose = self.hsub.getPose(cached=True)[0:2]
            self.postEvent("POSE", tuple(map(int, self.hsub.coordmap_lab2map(pose))))

            last_gui_update_time = self.timer_func()

        logging.debug("execute.py quitting...")

    # This function is necessary to prevent xmlrpcserver from catching
    # exceptions and eating the tracebacks
    def _dispatch(self, method, args):
        try:
            return getattr(self, method)(*args)
        except:
            traceback.print_exc()
            raise

class RedirectText:
    def __init__(self, event_handler):
        self.event_handler = event_handler

    def write(self, message):
        if message.strip() != "":
            self.event_handler("OTHER", message.strip())

    def flush(self):
        pass


####################################################
# Main function, run when called from command-line #
####################################################

def execute_main(listen_port=None, spec_file=None, aut_file=None, show_gui=False):
    logging.info("Hello. Let's do this!")

    # Create the XML-RPC server
    if listen_port is None:
        # Search for a port we can successfully bind to
        while True:
            listen_port = random.randint(10000, 65535)
            try:
                xmlrpc_server = SimpleXMLRPCServer(("127.0.0.1", listen_port), logRequests=False, allow_none=True)
            except socket.error as e:
                pass
            else:
                break
    else:
        xmlrpc_server = SimpleXMLRPCServer(("127.0.0.1", listen_port), logRequests=False, allow_none=True)

    # Create the execution context object
    e = LTLMoPExecutor()

    # Register functions with the XML-RPC server
    xmlrpc_server.register_instance(e)

    # Kick off the XML-RPC server thread
    XMLRPCServerThread = threading.Thread(target=xmlrpc_server.serve_forever)
    XMLRPCServerThread.daemon = True
    XMLRPCServerThread.start()
    logging.info("Executor listening for XML-RPC calls on http://127.0.0.1:{} ...".format(listen_port))

    # Start the GUI if necessary
    if show_gui:
        # Create a subprocess
        logging.info("Starting GUI window...")
        p_gui = subprocess.Popen([sys.executable, "-u", "-m", "lib.simGUI", str(listen_port)])

        # Wait for GUI to fully load, to make sure that
        # to make sure all messages are redirected
        e.externalEventTargetRegistered.wait()

    if spec_file is not None:
        # Tell executor to load spec & aut
        #if aut_file is None:
        #    aut_file = spec_file.rpartition('.')[0] + ".aut"
        e.initialize(spec_file, aut_file, firstRun=True)

    # Start the executor's main loop in this thread
    e.run()

    # Clean up on exit
    logging.info("Waiting for XML-RPC server to shut down...")
    xmlrpc_server.shutdown()
    XMLRPCServerThread.join()
    logging.info("XML-RPC server shutdown complete.  Goodbye.")


### Command-line argument parsing ###

if __name__ == "__main__":
    ### Check command-line arguments

    aut_file = None
    spec_file = None
    show_gui = True
    listen_port = None

    try:
        opts, args = getopt.getopt(sys.argv[1:], "hnp:a:s:", ["help", "no-gui", "xmlrpc-listen-port=", "aut-file=", "spec-file="])
    except getopt.GetoptError:
        logging.exception("Bad arguments")
        usage(sys.argv[0])
        sys.exit(2)

    for opt, arg in opts:
        if opt in ("-h", "--help"):
            usage(sys.argv[0])
            sys.exit()
        elif opt in ("-n", "--no-gui"):
            show_gui = False
        elif opt in ("-p", "--xmlrpc-listen-port"):
            try:
                listen_port = int(arg)
            except ValueError:
                logging.error("Invalid port '{}'".format(arg))
                sys.exit(2)
        elif opt in ("-a", "--aut-file"):
            aut_file = arg
        elif opt in ("-s", "--spec-file"):
            spec_file = arg

    execute_main(listen_port, spec_file, aut_file, show_gui)
