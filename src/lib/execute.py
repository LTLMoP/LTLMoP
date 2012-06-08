#!/usr/bin/env python

""" =================================================
    execute.py - Top-level hybrid controller executor
    =================================================

    This module executes a hybrid controller for a robot in a simulated or real environment.

    :Usage: ``execute.py [-hn] [-a automaton_file] [-s spec_file]``

    * The controlling automaton is imported from the specified ``automaton_file``.

    * The supporting handler modules (e.g. sensor, actuator, motion control, simulation environment initialization, etc)
      are loaded according to the settings provided in the specified ``spec_file``.

    * Unless otherwise specified with the ``-n`` or ``--no_gui`` option, a status/control window
      will also be opened for informational purposes.
"""

import sys, os, getopt, textwrap
import threading, subprocess, time
import fileMethods, regions, fsa, project
from numpy import *
from handlers.motionControl.__is_inside import is_inside
from socket import *

####################
# HELPER FUNCTIONS #
####################

def usage(script_name):
    """ Print command-line usage information. """

    print textwrap.dedent("""\
                              Usage: %s [-hn] [-a automaton_file] [-s spec_file]

                              -h, --help:
                                  Display this message
                              -n, --no-gui:
                                  Do not show status/control window
                              -a FILE, --aut-file FILE:
                                  Load automaton from FILE
                              -s FILE, --spec-file FILE:
                                  Load experiment configuration from FILE """ % script_name)

####################
# THREAD FUNCTIONS #
####################

def guiListen():
    """
    Processes messages from the GUI window, and reacts accordingly
    """

    global guiListenInitialized, runFSA

    # Set up socket for communication from simGUI
    host = 'localhost'
    portFrom = 9562
    buf = 1024
    addrFrom = (host,portFrom)
    UDPSockFrom = socket(AF_INET,SOCK_DGRAM)
    UDPSockFrom.setsockopt(SOL_SOCKET,SO_REUSEADDR,1)
    UDPSockFrom.bind(addrFrom)

    while 1:
        # Wait for and receive a message from the subwindow
        input,addrFrom = UDPSockFrom.recvfrom(buf)

        if input == '':  # EOF indicates that the connection has been destroyed
            print "GUI listen thread is shutting down!"
            break

        input = input.strip()

        # Check for the initialization signal, if necessary
        if not guiListenInitialized and input == "Hello!":
            guiListenInitialized = True
            continue

        # Do what the GUI tells us, lest we anger it
        if input == "START":
            runFSA = True
        elif input == "PAUSE":
            runFSA = False
            print "PAUSED."
        else:
            print "WARNING: Unknown command received from GUI: " + input

#########################
# MAIN EXECUTION THREAD #
#########################

def main(argv):
    """ Main function; run automatically when called from command-line """

    ################################
    # Check command-line arguments #
    ################################

    aut_file = None
    spec_file = None
    show_gui = True

    try:
        opts, args = getopt.getopt(argv[1:], "hna:s:", ["help", "no-gui", "aut-file=", "spec-file="])
    except getopt.GetoptError, err:
        print str(err)
        usage(argv[0])
        sys.exit(2)

    for opt, arg in opts:
        if opt in ("-h", "--help"):
            usage(argv[0])
            sys.exit()
        elif opt in ("-n", "--no-gui"):
            show_gui = False
        elif opt in ("-a", "--aut-file"):
            aut_file = arg
        elif opt in ("-s", "--spec-file"):
            spec_file = arg

    if aut_file is None:
        print "ERROR: Automaton file needs to be specified."
        usage(argv[0])
        sys.exit(2)

    if spec_file is None:
        print "ERROR: Specification file needs to be specified."
        usage(argv[0])
        sys.exit(2)

    print "\n[ LTLMOP HYBRID CONTROLLER EXECUTION MODULE ]\n"
    print "Hello. Let's do this!\n"

    ############################
    # Load configuration files #
    ############################

    proj = project.Project()
    proj.loadProject(spec_file)
    proj.rfiold = proj.rfi
    proj.rfi = proj.loadRegionFile(decomposed=True)
    print proj.rfi

    ##########################
    # Initialize each module #
    ##########################

    # Import the relevant handlers
    print "Importing handler functions..."

    proj.importHandlers()

    ############################
    # Start status/control GUI #
    ############################

    if show_gui:
        global runFSA, fd_gui_output, fd_gui_input, guiListenInitialized  # For sharing with thread

        runFSA = False  # Start out paused
        guiListenInitialized = False

        # Create a subprocess
        print "Starting GUI window and listen thread..."
        p_gui = subprocess.Popen(["python",os.path.join(proj.ltlmop_root, "lib", "simGUI.py")], stderr=subprocess.PIPE, stdin=subprocess.PIPE)


        # Create new thread to communicate with subwindow
        guiListenThread = threading.Thread(target = guiListen)
        guiListenThread.start()

        # Set up socket for communication to simGUI
        host = 'localhost'
        portTo = 9563
        buf = 1024
        addrTo = (host,portTo)
        UDPSockTo = socket(AF_INET,SOCK_DGRAM)
        # Block until the GUI listener gets the go-ahead from the subwindow
        while not guiListenInitialized:
            time.sleep(0.05) # We need to sleep to give up the CPU

        # Tell GUI to load background image
        message = "BG:" + proj.getFilenamePrefix() + ".spec"
        UDPSockTo.sendto(message,addrTo)

        # Redirect all output to the log
        redir = RedirectText(UDPSockTo, addrTo)

        sys.stdout = redir
        sys.stderr = redir

    #######################
    # Load automaton file #
    #######################

    print "Loading automaton..."

    FSA = fsa.Automaton(proj)


    success = FSA.loadFile(aut_file, proj.enabled_sensors, proj.enabled_actuators, proj.all_customs)
    if not success: return

    #############################
    # Begin automaton execution #
    #############################


    last_gui_update_time = 0

    ### Wait for the initial start command

    if show_gui:
        while not runFSA:
            time.sleep(0.05) # We need to sleep to give up the CPU
    else:
        raw_input('Press enter to begin...')
        runFSA = True

    ### Figure out where we should start from

    pose = proj.h_instance['pose'].getPose()

    init_region = None

    for i, r in enumerate(proj.rfi.regions):
        #pointArray = [proj.coordmap_map2lab(x) for x in r.getPoints()]
        #vertices = mat(pointArray).T

        #if is_inside([pose[0], pose[1]], vertices):
        if r.objectContainsPoint(*proj.coordmap_lab2map(pose)):
            init_region = i
            break

    if init_region is None:
        print "Initial pose of ", pose, "not inside any region!"
        sys.exit(1)

    print "Starting from initial region: " + proj.rfi.regions[init_region].name

    ### Have the FSA find a valid initial state

    # Figure out our initially true outputs
    init_outputs = []
    for prop in proj.currentConfig.initial_truths:
        if prop not in proj.enabled_sensors:
            init_outputs.append(prop)

    init_state = FSA.chooseInitialState(init_region, init_outputs)

    if init_state is None:
        print "No suitable initial state found; unable to execute. Quitting..."
        sys.exit(-1)
    else:
        print "Starting from state %s." % init_state.name

    ### Get everything moving

    avg_freq = 0

    # Choose a timer func with maximum accuracy for given platform
    if sys.platform in ['win32', 'cygwin']:
        timer_func = time.clock
    else:
        timer_func = time.time

    while True:
        # Idle if we're not running
        while not runFSA:
            proj.h_instance['drive'].setVelocity(0,0)
            time.sleep(0.05) # We need to sleep to give up the CPU

        tic = timer_func()

        FSA.runIteration()

        toc = timer_func()

        # TODO: Possibly implement max rate-limiting?
        #while (toc - tic) < 0.05:
        #   time.sleep(0.01)
        #   toc = time.time()

        # Update GUI, no faster than 20Hz
        if show_gui and (time.time() - last_gui_update_time > 0.05):
            avg_freq = 0.9*avg_freq + 0.1*1/(toc-tic) # IIR filter
            UDPSockTo.sendto("Running at approximately %dHz..." % int(avg_freq),addrTo)
            pose = proj.h_instance['pose'].getPose(cached=True)[0:2]
            UDPSockTo.sendto("POSE:%d,%d" % tuple(map(int, proj.coordmap_lab2map(pose))),addrTo)

            last_gui_update_time = time.time()

class RedirectText:
    """
    A class that lets the output of a stream be directed into a socket.

    http://mail.python.org/pipermail/python-list/2007-June/445795.html
    """

    def __init__(self,s,addrTo):
        self.s = s
        self.addrTo = addrTo

    def write(self,string):
        self.s.sendto(string, self.addrTo)


if __name__ == "__main__":
    main(sys.argv)

