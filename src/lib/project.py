#!/usr/bin/env python

""" ================================================
    project.py - Abstraction layer for project files
    ================================================

    This module exposes an object that allows for simplified loading of the
    various files included in a single project.
"""

# TODO: Document better

import os, sys
import fileMethods, regions
from numpy import *
import logging
import globalConfig

class Project:
    """
    A project object.
    """

    def __init__(self):
        self.project_basename = None
        self.project_root = None
        self.spec_data = None
        self.silent = False
        self.regionMapping = None
        self.rfi = None
        self.specText = ""
        self.all_sensors = []
        self.enabled_sensors = []
        self.all_actuators = []
        self.enabled_actuators = []
        self.all_customs = []
        self.internal_props = []
        self.current_config = ""
        self.shared_data = {}  # This is for storing things like server connection objects, etc.

        self.h_instance = {'init':{},'pose':None,'locomotionCommand':None,'motionControl':None,'drive':None,'sensor':{},'actuator':{}}

        # Compilation options (with defaults)
        self.compile_options = {"convexify": True,  # Decompose workspace into convex regions
                                "fastslow": False,  # Enable "fast-slow" synthesis algorithm
                                "symbolic": False,  # Use BDDs instead of explicit-state strategies
                                "decompose": True,  # Create regions for free space and region overlaps (required for Locative Preposition support)
                                "use_region_bit_encoding": True, # Use a vector of "bitX" propositions to represent regions, for efficiency
                                "synthesizer": "jtlv", # Name of synthesizer to use ("jtlv" or "slugs")
                                "parser": "structured"}  # Spec parser: SLURP ("slurp"), structured English ("structured"), or LTL ("ltl")

        self.ltlmop_root = globalConfig.get_ltlmop_root()

    def setSilent(self, silent):
        self.silent = silent

    def loadRegionMapping(self):
        """
        Takes the region mapping data and returns region mapping dictionary.
        """

        if self.spec_data is None:
            logging.error("Cannot load region mapping data before loading a spec file")
            return None

        try:
            mapping_data = self.spec_data['SPECIFICATION']['RegionMapping']
        except KeyError:
            logging.warning("Region mapping data undefined")
            return None

        if len(mapping_data) == 0:
            logging.warning("Region mapping data is empty")
            return None

        regionMapping = {}
        for line in mapping_data:
            oldRegionName, newRegionList = line.split('=')
            regionMapping[oldRegionName.strip()] = [n.strip() for n in newRegionList.split(',')]

        return regionMapping

    def loadRegionFile(self, decomposed=False):
        """
        Returns a Region File Interface object corresponding to the regions file referenced in the spec file
        """

        #### Load in the region file

        if decomposed:
            regf_name = self.getFilenamePrefix() + "_decomposed.regions"
        else:
            try:
                regf_name = os.path.join(self.project_root, self.spec_data['SETTINGS']['RegionFile'][0])
            except (IndexError, KeyError):
                logging.warning("Region file undefined")
                return None

        logging.info("Loading region file %s..." % regf_name)
        rfi = regions.RegionFileInterface()

        if not rfi.readFile(regf_name):
            if not self.silent:
                logging.error("Could not load region file %s!"  % regf_name)
                if decomposed:
                    logging.error("Are you sure you compiled your specification?")
            return None

        logging.info("Found definitions for %d regions." % len(rfi.regions))

        return rfi

    def loadSpecFile(self, spec_file):
        # Figure out where we should be looking for files, based on the spec file name & location
        self.project_root = os.path.abspath(os.path.dirname(spec_file))
        self.project_basename, ext = os.path.splitext(os.path.basename(spec_file))


        ### Load in the specification file
        logging.info("Loading specification file %s..." % spec_file)
        spec_data = fileMethods.readFromFile(spec_file)

        if spec_data is None:
            logging.warning("Failed to load specification file")
            return None

        try:
            self.specText = '\n'.join(spec_data['SPECIFICATION']['Spec'])
        except KeyError:
            logging.warning("Specification text undefined")

        if 'CompileOptions' in spec_data['SETTINGS']:
            for l in spec_data['SETTINGS']['CompileOptions']:
                if ":" not in l:
                    continue

                k,v = l.split(":", 1)
                if k.strip().lower() in ("parser", "synthesizer"):
                    self.compile_options[k.strip().lower()] = v.strip().lower()
                else:
                    # convert to boolean if not a parser type
                    self.compile_options[k.strip().lower()] = (v.strip().lower() in ['true', 't', '1'])

        return spec_data

    def writeSpecFile(self, filename=None):
        if filename is None:
            # Default to same filename as we loaded from
            filename = os.path.join(self.project_root, self.project_basename + ".spec")
        else:
            # Update our project paths based on the new filename
            self.project_root = os.path.dirname(os.path.abspath(filename))
            self.project_basename, ext = os.path.splitext(os.path.basename(filename))

        data = {}

        data['SPECIFICATION'] = {"Spec": self.specText}

        if self.regionMapping is not None:
            data['SPECIFICATION']['RegionMapping'] = [rname + " = " + ', '.join(rlist) for
                                                      rname, rlist in self.regionMapping.iteritems()]

        data['SETTINGS'] = {"Sensors": [p + ", " + str(int(p in self.enabled_sensors)) for p in self.all_sensors if not (p.endswith('_rc') or p.endswith('_ac'))],  #TODO: don't need this later
                            "Actions": [p + ", " + str(int(p in self.enabled_actuators)) for p in self.all_actuators],
                            "Customs": self.all_customs}

        if self.current_config is not "":
            data['SETTINGS']['CurrentConfigName'] = self.current_config

        data['SETTINGS']['CompileOptions'] = "\n".join(["%s: %s" % (k, str(v)) for k,v in self.compile_options.iteritems()])

        if self.rfi is not None:
            # Save the path to the region file as relative to the spec file
            # FIXME: relpath has case sensitivity problems on OS X
            data['SETTINGS']['RegionFile'] = os.path.normpath(os.path.relpath(self.rfi.filename, self.project_root))

        comments = {"FILE_HEADER": "This is a specification definition file for the LTLMoP toolkit.\n" +
                                   "Format details are described at the beginning of each section below.",
                    "RegionFile": "Relative path of region description file",
                    "Sensors": "List of sensor propositions and their state (enabled = 1, disabled = 0)",
                    "Actions": "List of action propositions and their state (enabled = 1, disabled = 0)",
                    "Customs": "List of custom propositions",
                    "Spec": "Specification in structured English",
                    "RegionMapping": "Mapping between region names and their decomposed counterparts"}

        fileMethods.writeToFile(filename, data, comments)


    def loadProject(self, spec_file):
        """
        Because the spec_file contains references to all other project files, this is all we
        need to know in order to load everything in.
        """

        self.spec_data = self.loadSpecFile(spec_file)

        if self.spec_data is None:
            return False

        try:
            self.current_config = self.spec_data['SETTINGS']['CurrentConfigName'][0]
        except (KeyError, IndexError):
            logging.warning("No experiment configuration defined")

        self.regionMapping = self.loadRegionMapping()
        self.rfi = self.loadRegionFile()
        self.determineEnabledPropositions()
        
        ## creates lists of regions and actuators completed if we are using fastslow
        if self.compile_options['fastslow']:
            # for region completion sensors
            self.rfi.regionsCompleted = self.populateCompletedPropositions([str(x.name) for x in self.rfi.regions if not "boundary" in x.name])
            self.all_sensors.extend(self.rfi.regionsCompleted)
            self.enabled_sensors.extend(self.rfi.regionsCompleted)
            
            # for actuator completion sensors
            self.enabled_actuatorsCompleted = self.populateCompletedPropositions(self.enabled_actuators,"_ac")
            self.all_sensors.extend(self.enabled_actuatorsCompleted)
            self.enabled_sensors.extend(self.enabled_actuatorsCompleted)

        return True
        
    def populateCompletedPropositions(self, propList, suffix = '_rc'):
        """
        Takes in a list of proposition and populates a list that has the same length with each element added 
        a suffix after
        """
        completedPropList = [x+suffix for x in propList]
        return completedPropList
        
    def determineEnabledPropositions(self):
        """
        Populate lists ``all_sensors``, ``enabled_sensors``, etc.
        """

        # Figure out what sensors are enabled
        self.all_sensors = []
        self.enabled_sensors = []
        for line in self.spec_data['SETTINGS']['Sensors']:
            sensor, val = line.split(',')
            self.all_sensors.append(sensor.strip())
            if int(val) == 1:
                self.enabled_sensors.append(sensor.strip())

        # Figure out what actuators are enabled
        self.all_actuators = []
        self.enabled_actuators = []
        for line in self.spec_data['SETTINGS']['Actions']:
            act, val = line.split(',')
            self.all_actuators.append(act.strip())
            if int(val) == 1:
                self.enabled_actuators.append(act.strip())

        # Figure out what the custom propositions are
        self.all_customs = self.spec_data['SETTINGS']['Customs']

    def getFilenamePrefix(self):
        """ Returns the full path of most project files, minus the extension.

            For example, if the spec file of this project is ``/home/ltlmop/examples/test/test.spec``
            then this function will return ``/home/ltlmop/examples/test/test``
        """
        return os.path.join(self.project_root, self.project_basename)

    def getStrategyFilename(self):
        """ Returns the full path of the file that should contain the strategy
            for this specification. """

        return self.getFilenamePrefix() + ('.bdd' if self.compile_options["symbolic"] else '.aut')



