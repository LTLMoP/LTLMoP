import os, sys
import re
import time
import math
import subprocess
import numpy
import glob
import StringIO
import logging

from multiprocessing import Pool

import project
import regions
import parseLP
from createJTLVinput import createLTLfile, createSMVfile, createTopologyFragment, createInitialRegionFragment
from parseEnglishToLTL import bitEncoding, replaceRegionName, createStayFormula
import fsa
from copy import deepcopy
from cores.coreUtils import *
import handlerSubsystem

from asyncProcesses import AsynchronousProcessThread

import strategy

# Hack needed to ensure there's only one
_SLURP_SPEC_GENERATOR = None


class SpecCompiler(object):
    def __init__(self, spec_filename=None):
        self.proj = project.Project()
        self.synthesis_subprocess = None

        if spec_filename is not None:
            self.loadSpec(spec_filename)

    def loadSpec(self,spec_filename):
        """
        Load the project object
        """
        self.proj.loadProject(spec_filename)

        # Check to make sure this project is complete
        if self.proj.rfi is None:
            logging.warning("Please define regions before compiling.")
            return

        # Remove comments
        self.specText = re.sub(r"#.*$", "", self.proj.specText, flags=re.MULTILINE)

        if self.specText.strip() == "":
            logging.warning("Please write a specification before compiling.")
            return

    def loadSimpleSpec(self,text="", regionList=[], sensors=[], actuators=[], customs=[], adj=[], outputfile=""):
        """
        Load a simple spec given by the arguments without reading from a spec file

        For Slurp

        region, sensors, actuators, customs are lists of strings representing props
        adj is a list of tuples [(region1,region2),...]
        """

        if outputfile == "":
            logging.error("Need to specify output filename")
            return

        self.proj.compile_options['decompose'] = False
        self.proj.project_root = os.path.abspath(os.path.dirname(os.path.expanduser(outputfile)))
        self.proj.project_basename, ext = os.path.splitext(os.path.basename(outputfile))
        self.proj.specText=text
        # construct a list of region objects with given names
        self.proj.rfi = regions.RegionFileInterface()
        for rname in regionList:
            self.proj.rfi.regions.append(regions.Region(name=rname))

        self.proj.enabled_sensors = sensors
        self.proj.enabled_actuators = actuators
        self.proj.all_customs = customs

        # construct adjacency matrix
        self.proj.rfi.transitions= [[[] for j in range(len(self.proj.rfi.regions))] for i in range(len(self.proj.rfi.regions))]
        for tran in adj:
            idx0 = self.proj.rfi.indexOfRegionWithName(tran[0])
            idx1 = self.proj.rfi.indexOfRegionWithName(tran[1])
            self.proj.rfi.transitions[idx0][idx1] = [(0,0)] # fake trans face
            self.proj.rfi.transitions[idx1][idx0] = [(0,0)]

    def _decompose(self):
        self.parser = parseLP.parseLP()
        self.parser.main(self.proj.getFilenamePrefix() + ".spec")

        # Remove all references to any obstacle regions at this point
        for r in self.proj.rfi.regions:
            if r.isObstacle:
                # Delete corresponding decomposed regions
                for sub_r in self.parser.proj.regionMapping[r.name]:
                    del self.parser.proj.rfi.regions[self.parser.proj.rfi.indexOfRegionWithName(sub_r)]

                    # Remove decomposed region from any overlapping mappings
                    for k,v in self.parser.proj.regionMapping.iteritems():
                        if k == r.name: continue
                        if sub_r in v:
                            v.remove(sub_r)

                # Remove mapping for the obstacle region
                del self.parser.proj.regionMapping[r.name]

        #self.proj.rfi.regions = filter(lambda r: not (r.isObstacle or r.name == "boundary"), self.proj.rfi.regions)

        # save the regions into new region file
        filename = self.proj.getFilenamePrefix() + '_decomposed.regions'

        # FIXME: properly support obstacles in non-decomposed maps?
        if self.proj.compile_options["decompose"]:
            self.parser.proj.rfi.recalcAdjacency()

        self.parser.proj.rfi.writeFile(filename)


        self.proj.regionMapping = self.parser.proj.regionMapping
        self.proj.writeSpecFile()

    def _writeSMVFile(self):
        if self.proj.compile_options["decompose"]:
            numRegions = len(self.parser.proj.rfi.regions)
        else:
            numRegions = len(self.proj.rfi.regions)
        sensorList = self.proj.enabled_sensors
        robotPropList = self.proj.enabled_actuators + self.proj.all_customs + self.proj.internal_props

        # Add in regions as robot outputs
        if self.proj.compile_options["use_region_bit_encoding"]:
            robotPropList.extend(["bit"+str(i) for i in range(0,int(numpy.ceil(numpy.log2(numRegions))))])
        else:
            if self.proj.compile_options["decompose"]:
                robotPropList.extend([r.name for r in self.parser.proj.rfi.regions])
            else:
                robotPropList.extend([r.name for r in self.proj.rfi.regions])

        self.propList = sensorList + robotPropList

        createSMVfile(self.proj.getFilenamePrefix(), sensorList, robotPropList)

    def _writeLTLFile(self):

        self.LTL2SpecLineNumber = None

        #regionList = [r.name for r in self.parser.proj.rfi.regions]
        regionList = [r.name for r in self.proj.rfi.regions]
        sensorList = deepcopy(self.proj.enabled_sensors)
        robotPropList = self.proj.enabled_actuators + self.proj.all_customs

        text = self.proj.specText

        response = None

        # Create LTL using selected parser
        # TODO: rename decomposition object to something other than 'parser'
        if self.proj.compile_options["parser"] == "slurp":
            # default to no region tags if no simconfig is defined, so we can compile without
            if self.proj.current_config == "":
                region_tags = {}
            else:
                self.hsub = handlerSubsystem.HandlerSubsystem(None, self.proj.project_root)
                config, success = self.hsub.loadConfigFile(self.proj.current_config)
                if success: self.hsub.configs.append(config)
                self.hsub.setExecutingConfig(self.proj.current_config)

                region_tags = self.hsub.executing_config.region_tags

            # Hack: We need to make sure there's only one of these
            global _SLURP_SPEC_GENERATOR

            # Make a new specgenerator and have it process the text
            if not _SLURP_SPEC_GENERATOR:
                # Add SLURP to path for import
                p = os.path.dirname(os.path.abspath(__file__))
                sys.path.append(os.path.join(p, "..", "etc", "SLURP"))
                from ltlbroom.specgeneration import SpecGenerator
                _SLURP_SPEC_GENERATOR = SpecGenerator()

            # Filter out regions it shouldn't know about
            filtered_regions = [region.name for region in self.proj.rfi.regions
                                if not (region.isObstacle or region.name.lower() == "boundary")]
            LTLspec_env, LTLspec_sys, self.proj.internal_props, internal_sensors, results, responses, traceback = \
                _SLURP_SPEC_GENERATOR.generate(text, sensorList, filtered_regions, robotPropList, region_tags)

            oldspec_env = LTLspec_env
            oldspec_sys = LTLspec_sys

            for ln, result in enumerate(results):
                if not result:
                    logging.warning("Could not parse the sentence in line {0}".format(ln))

            # Abort compilation if there were any errors
            if not all(results):
                return None, None, responses

            # Add in the sensors so they go into the SMV and spec files
            for s in internal_sensors:
                if s not in sensorList:
                    sensorList.append(s)
                    self.proj.all_sensors.append(s)
                    self.proj.enabled_sensors.append(s)

            # Conjoin all the spec chunks
            LTLspec_env = '\t\t' + ' & \n\t\t'.join(LTLspec_env)
            LTLspec_sys = '\t\t' + ' & \n\t\t'.join(LTLspec_sys)

            if self.proj.compile_options["decompose"]:
                # substitute decomposed region names
                for r in self.proj.rfi.regions:
                    if not (r.isObstacle or r.name.lower() == "boundary"):
                        LTLspec_env = re.sub('\\bs\.' + r.name + '\\b', "("+' | '.join(["s."+x for x in self.parser.proj.regionMapping[r.name]])+")", LTLspec_env)
                        LTLspec_env = re.sub('\\be\.' + r.name + '\\b', "("+' | '.join(["e."+x for x in self.parser.proj.regionMapping[r.name]])+")", LTLspec_env)
                        LTLspec_sys = re.sub('\\bs\.' + r.name + '\\b', "("+' | '.join(["s."+x for x in self.parser.proj.regionMapping[r.name]])+")", LTLspec_sys)
                        LTLspec_sys = re.sub('\\be\.' + r.name + '\\b', "("+' | '.join(["e."+x for x in self.parser.proj.regionMapping[r.name]])+")", LTLspec_sys)

            response = responses

        elif self.proj.compile_options["parser"] == "ltl":
            # delete comments
            text = re.sub(r"#.*$", "", text, flags=re.MULTILINE)

            # split into env and sys parts (by looking for a line of just dashes in between)
            LTLspec_env, LTLspec_sys = re.split(r"^\s*-+\s*$", text, maxsplit=1, flags=re.MULTILINE)

            # split into subformulas
            LTLspec_env = re.split(r"(?:[ \t]*[\n\r][ \t]*)+", LTLspec_env)
            LTLspec_sys = re.split(r"(?:[ \t]*[\n\r][ \t]*)+", LTLspec_sys)

            # remove any empty initial entries (HACK?)
            while '' in LTLspec_env:
                LTLspec_env.remove('')
            while '' in LTLspec_sys:
                LTLspec_sys.remove('')

            # automatically conjoin all the subformulas
            LTLspec_env = '\t\t' + ' & \n\t\t'.join(LTLspec_env)
            LTLspec_sys = '\t\t' + ' & \n\t\t'.join(LTLspec_sys)

            if self.proj.compile_options["decompose"]:
                # substitute decomposed region
                for r in self.proj.rfi.regions:
                    if not (r.isObstacle or r.name.lower() == "boundary"):
                        LTLspec_env = re.sub('\\b(?:s\.)?' + r.name + '\\b', "("+' | '.join(["s."+x for x in self.parser.proj.regionMapping[r.name]])+")", LTLspec_env)
                        LTLspec_sys = re.sub('\\b(?:s\.)?' + r.name + '\\b', "("+' | '.join(["s."+x for x in self.parser.proj.regionMapping[r.name]])+")", LTLspec_sys)
            else:
                for r in self.proj.rfi.regions:
                    if not (r.isObstacle or r.name.lower() == "boundary"):
                        LTLspec_env = re.sub('\\b(?:s\.)?' + r.name + '\\b', "s."+r.name, LTLspec_env)
                        LTLspec_sys = re.sub('\\b(?:s\.)?' + r.name + '\\b', "s."+r.name, LTLspec_sys)

            traceback = [] # HACK: needs to be something other than None
        elif self.proj.compile_options["parser"] == "structured":
            import parseEnglishToLTL

            if self.proj.compile_options["decompose"]:
                # substitute the regions name in specs
                for m in re.finditer(r'near (?P<rA>\w+)', text):
                    text=re.sub(r'near (?P<rA>\w+)', "("+' or '.join(["s."+r for r in self.parser.proj.regionMapping['near$'+m.group('rA')+'$'+str(50)]])+")", text)
                for m in re.finditer(r'within (?P<dist>\d+) (from|of) (?P<rA>\w+)', text):
                    text=re.sub(r'within ' + m.group('dist')+' (from|of) '+ m.group('rA'), "("+' or '.join(["s."+r for r in self.parser.proj.regionMapping['near$'+m.group('rA')+'$'+m.group('dist')]])+")", text)
                for m in re.finditer(r'between (?P<rA>\w+) and (?P<rB>\w+)', text):
                    text=re.sub(r'between ' + m.group('rA')+' and '+ m.group('rB'),"("+' or '.join(["s."+r for r in self.parser.proj.regionMapping['between$'+m.group('rA')+'$and$'+m.group('rB')+"$"]])+")", text)

                # substitute decomposed region
                for r in self.proj.rfi.regions:
                    if not (r.isObstacle or r.name.lower() == "boundary"):
                        text = re.sub('\\b' + r.name + '\\b', "("+' | '.join(["s."+x for x in self.parser.proj.regionMapping[r.name]])+")", text)

                regionList = ["s."+x.name for x in self.parser.proj.rfi.regions]
            else:
                for r in self.proj.rfi.regions:
                    if not (r.isObstacle or r.name.lower() == "boundary"):
                        text = re.sub('\\b' + r.name + '\\b', "s."+r.name, text)

                regionList = ["s."+x.name for x in self.proj.rfi.regions]

            spec, traceback, failed, self.LTL2SpecLineNumber, self.proj.internal_props = parseEnglishToLTL.writeSpec(text, sensorList, regionList, robotPropList)

            # Abort compilation if there were any errors
            if failed:
                return None, None, None

            LTLspec_env = spec["EnvInit"] + spec["EnvTrans"] + spec["EnvGoals"]
            LTLspec_sys = spec["SysInit"] + spec["SysTrans"] + spec["SysGoals"]

        else:
            logging.error("Parser type '{0}' not currently supported".format(self.proj.compile_options["parser"]))
            return None, None, None

        if self.proj.compile_options["decompose"]:
            regionList = [x.name for x in self.parser.proj.rfi.regions]
        else:
            regionList = [x.name for x in self.proj.rfi.regions]

        if self.proj.compile_options["use_region_bit_encoding"]:
            # Define the number of bits needed to encode the regions
            numBits = int(math.ceil(math.log(len(regionList),2)))

            # creating the region bit encoding
            bitEncode = bitEncoding(len(regionList),numBits)
            currBitEnc = bitEncode['current']
            nextBitEnc = bitEncode['next']

            # switch to bit encodings for regions
            LTLspec_env = replaceRegionName(LTLspec_env, bitEncode, regionList)
            LTLspec_sys = replaceRegionName(LTLspec_sys, bitEncode, regionList)

            if self.LTL2SpecLineNumber is not None:
                for k in self.LTL2SpecLineNumber.keys():
                    new_k = replaceRegionName(k, bitEncode, regionList)
                    if new_k != k:
                        self.LTL2SpecLineNumber[new_k] = self.LTL2SpecLineNumber[k]
                        del self.LTL2SpecLineNumber[k]

        if self.proj.compile_options["decompose"]:
            adjData = self.parser.proj.rfi.transitions
        else:
            adjData = self.proj.rfi.transitions

        # Store some data needed for later analysis
        self.spec = {}
        if self.proj.compile_options["decompose"]:
            self.spec['Topo'] = createTopologyFragment(adjData, self.parser.proj.rfi.regions, use_bits=self.proj.compile_options["use_region_bit_encoding"])
        else:
            self.spec['Topo'] = createTopologyFragment(adjData, self.proj.rfi.regions, use_bits=self.proj.compile_options["use_region_bit_encoding"])

        # Substitute any macros that the parsers passed us
        LTLspec_env = self.substituteMacros(LTLspec_env)
        LTLspec_sys = self.substituteMacros(LTLspec_sys)

        # If we are not using bit-encoding, we need to
        # explicitly encode a mutex for regions
        if not self.proj.compile_options["use_region_bit_encoding"]:
            # DNF version (extremely slow for core-finding)
            #mutex = "\n\t&\n\t []({})".format(" | ".join(["({})".format(" & ".join(["s."+r2.name if r is r2 else "!s."+r2.name for r2 in self.parser.proj.rfi.regions])) for r in self.parser.proj.rfi.regions]))

            if self.proj.compile_options["decompose"]:
                region_list = self.parser.proj.rfi.regions
            else:
                region_list = self.proj.rfi.regions

            # Almost-CNF version
            exclusions = []
            for i, r1 in enumerate(region_list):
                for r2 in region_list[i+1:]:
                    exclusions.append("!(s.{} & s.{})".format(r1.name, r2.name))
            mutex = "\n&\n\t []({})".format(" & ".join(exclusions))
            LTLspec_sys += mutex

        self.spec.update(self.splitSpecIntoComponents(LTLspec_env, LTLspec_sys))

        # Add in a fragment to make sure that we start in a valid region
        if self.proj.compile_options["decompose"]:
            self.spec['InitRegionSanityCheck'] = createInitialRegionFragment(self.parser.proj.rfi.regions, use_bits=self.proj.compile_options["use_region_bit_encoding"])
        else:
            self.spec['InitRegionSanityCheck'] = createInitialRegionFragment(self.proj.rfi.regions, use_bits=self.proj.compile_options["use_region_bit_encoding"])
        LTLspec_sys += "\n&\n" + self.spec['InitRegionSanityCheck']

        LTLspec_sys += "\n&\n" + self.spec['Topo']

        createLTLfile(self.proj.getFilenamePrefix(), LTLspec_env, LTLspec_sys)

        if self.proj.compile_options["parser"] == "slurp":
            self.reversemapping = {self.postprocessLTL(line,sensorList,robotPropList).strip():line.strip() for line in oldspec_env + oldspec_sys}
            self.reversemapping[self.spec['Topo'].replace("\n","").replace("\t","").lstrip().rstrip("\n\t &")] = "TOPOLOGY"

        #for k,v in self.reversemapping.iteritems():
        #    print "{!r}:{!r}".format(k,v)

        return self.spec, traceback, response

    def substituteMacros(self, text):
        """
        Replace any macros passed to us by the parser.  In general, this is only necessary in cases
        where bitX propositions are needed, since the parser is not supposed to know about them.
        """

        # This creates a mirrored copy of topological constraints for the target we are following
        if "FOLLOW_SENSOR_CONSTRAINTS" in text:
            if not self.proj.compile_options["use_region_bit_encoding"]:
                logging.warning("Currently, bit encoding must be enabled for follow sensor")
            else:
                env_topology = self.spec['Topo'].replace("s.bit", "e.sbit")
                initreg_formula = createInitialRegionFragment(self.parser.proj.rfi.regions, use_bits=True).replace("s.bit", "e.sbit")

                sensorBits = ["sbit{}".format(i) for i in range(0,int(numpy.ceil(numpy.log2(len(self.parser.proj.rfi.regions)))))]
                for p in sensorBits:
                    if p not in self.proj.enabled_sensors:
                        self.proj.enabled_sensors.append(p)
                    if p not in self.proj.all_sensors:
                        self.proj.all_sensors.append(p)

                text = text.replace("FOLLOW_SENSOR_CONSTRAINTS", env_topology + "\n&\n" + initreg_formula)

        # Stay-there macros.  This can be done using just region names, but is much more concise
        # using bit encodings (and thus much faster to encode as CNF)

        if "STAY_THERE" in text:
            if self.proj.compile_options["decompose"]:
                text = text.replace("STAY_THERE", createStayFormula([r.name for r in self.parser.proj.rfi.regions], use_bits=self.proj.compile_options["use_region_bit_encoding"]))
            else:
                text = text.replace("STAY_THERE", createStayFormula([r.name for r in self.proj.rfi.regions], use_bits=self.proj.compile_options["use_region_bit_encoding"]))

        if "TARGET_IS_STATIONARY" in text:
            text = text.replace("TARGET_IS_STATIONARY", createStayFormula([r.name for r in self.parser.proj.rfi.regions], use_bits=self.proj.compile_options["use_region_bit_encoding"]).replace("s.","e.s"))

        return text


    def postprocessLTL(self, text, sensorList, robotPropList):
        # TODO: make everything use this
        if self.proj.compile_options["decompose"]:
            # substitute decomposed region names
            for r in self.proj.rfi.regions:
                if not (r.isObstacle or r.name.lower() == "boundary"):
                    text = re.sub('\\bs\.' + r.name + '\\b', "("+' | '.join(["s."+x for x in self.parser.proj.regionMapping[r.name]])+")", text)
                    text = re.sub('\\be\.' + r.name + '\\b', "("+' | '.join(["e."+x for x in self.parser.proj.regionMapping[r.name]])+")", text)

        if self.proj.compile_options["decompose"]:
            regionList = [x.name for x in self.parser.proj.rfi.regions]
        else:
            regionList = [x.name for x in self.proj.rfi.regions]

        # Define the number of bits needed to encode the regions
        numBits = int(math.ceil(math.log(len(regionList),2)))

        # creating the region bit encoding
        bitEncode = bitEncoding(len(regionList),numBits)
        currBitEnc = bitEncode['current']
        nextBitEnc = bitEncode['next']

        # switch to bit encodings for regions
        if self.proj.compile_options["use_region_bit_encoding"]:
            text = replaceRegionName(text, bitEncode, regionList)

        text = self.substituteMacros(text)

        return text

    def splitSpecIntoComponents(self, env, sys):
        spec = {}

        for agent, text in (("env", env), ("sys", sys)):
            for line in text.split("\n"):
                if line.strip() == '': continue

                if "[]<>" in line:
                    linetype = "goals"
                elif "[]" in line:
                    linetype = "trans"
                else:
                    linetype = "init"

                key = agent.title()+linetype.title()
                if key not in spec:
                    spec[key] = ""

                spec[key] += line + "\n"

        return spec

    def _checkForEmptyGaits(self):
        from simulator.ode.ckbot import CKBotLib

        # Initialize gait library
        self.library = CKBotLib.CKBotLib()

        err = 0
        libs = self.library
        libs.readLibe()
		# Check that each individual trait has a corresponding config-gait pair
        robotPropList = self.proj.enabled_actuators + self.proj.all_customs
        for act in robotPropList:
            act = act.strip("u's.")
            if act[0] == "T":
                act = act.strip("T_")
                #print act
                words = act.split("_and_")
                #print words
                config = libs.findGait(words)
                #print config
                if type(config) == type(None):
                    err_message = "No config-gait pair for actuator T_" + act + "\n"
                    logging.warning(err_message)
                    err = 1

    def _getSlugsCommand(self):
        slugs_path = os.path.join(self.proj.ltlmop_root, "etc", "slugs", "src", "slugs")

        # Check that slugs is compiled
        if not os.path.exists(slugs_path):
            # TODO: automatically compile for the user
            raise RuntimeError("Please compile the synthesis code first.  For instructions, see etc/slugs/README.md.")

        cmd = [slugs_path, "--sysInitRoboticsSemantics", self.proj.getFilenamePrefix() + ".slugsin"]

        return cmd

    def _getGROneCommand(self, module):
        jtlv_path = os.path.join(self.proj.ltlmop_root, "etc", "jtlv")

        # Check that GROneMain, etc. is compiled
        if not os.path.exists(os.path.join(jtlv_path, "GROne", "GROneMain.class")):
            # TODO: automatically compile for the user
            raise RuntimeError("The Java synthesis code does not appear to be compiled yet.  Please run dist/setup.py before using LTLMoP.")

        # Windows uses a different delimiter for the java classpath
        delim = ";" if os.name == "nt" else ":"

        classpath = delim.join([os.path.join(jtlv_path, "jtlv-prompt1.4.0.jar"),
                                os.path.join(jtlv_path, "GROne")])

        cmd = ["java", "-ea", "-Xmx512m", "-cp", classpath, module, self.proj.getFilenamePrefix() + ".smv", self.proj.getFilenamePrefix() + ".ltl"]

        return cmd

    def _autIsNonTrivial(self):
        """
        Check for a) empty automaton, or b) trivial initial-state automaton
         with no transitions
        (This can indicate unsatisfiable system initial conditions (case a),
         or an unsat environment (case b).)

        TODO: Do this in the Java code; it's super inefficient to
        load the whole aut just to check this.
        """

        if self.proj.compile_options["decompose"]:
            regions = self.parser.proj.rfi.regions
        else:
            regions = self.proj.rfi.regions

        region_domain = strategy.Domain("region", regions, strategy.Domain.B0_IS_MSB)
        strat = strategy.createStrategyFromFile(self.proj.getStrategyFilename(),
                                                self.proj.enabled_sensors,
                                                self.proj.enabled_actuators + self.proj.all_customs + [region_domain])

        nonTrivial = any([len(strat.findTransitionableStates({}, s)) > 0 for s in strat.iterateOverStates()])

        return nonTrivial

    def _analyze(self):
        if self.proj.compile_options["synthesizer"].lower() != "jtlv":
            raise RuntimeError("Analysis is currently only supported when using JTLV.")

        cmd = self._getGROneCommand("GROneDebug")
        if cmd is None:
            return (False, False, [], "")

        subp = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, close_fds=False)

        realizable = False
        unsat = False
        nonTrivial = False


        output = ""
        to_highlight = []
        for dline in subp.stdout:
            output += dline
            if "Specification is synthesizable!" in dline:   
                realizable = True            
                nonTrivial = self._autIsNonTrivial()
                if nonTrivial:
                    break

            ### Highlight sentences corresponding to identified errors ###

            # System unsatisfiability
            elif "System initial condition is unsatisfiable." in dline:
                to_highlight.append(("sys", "init"))
            elif "System transition relation is unsatisfiable." in dline:
                to_highlight.append(("sys", "trans"))
            elif "System highlighted goal(s) unsatisfiable" in dline:
                for l in (dline.strip()).split()[-1:]:
                    to_highlight.append(("sys", "goals", int(l)))
            elif "System highlighted goal(s) inconsistent with transition relation" in dline:
                to_highlight.append(("sys", "trans"))
                to_highlight.append(("sys", "init"))
                for l in (dline.strip()).split()[-1:]:
                    to_highlight.append(("sys", "goals", int(l)))
            elif "System initial condition inconsistent with transition relation" in dline:
                to_highlight.append(("sys", "init"))
                to_highlight.append(("sys", "trans"))

            # Environment unsatisfiability
            elif "Environment initial condition is unsatisfiable." in dline:
                to_highlight.append(("env", "init"))
            elif "Environment transition relation is unsatisfiable." in dline:
                to_highlight.append(("env", "trans"))
            elif "Environment highlighted goal(s) unsatisfiable" in dline:
                for l in (dline.strip()).split()[-1:]:
                    to_highlight.append(("env", "goals", int(l)))
            elif "Environment highlighted goal(s) inconsistent with transition relation" in dline:
                to_highlight.append(("env", "init"))
                to_highlight.append(("env", "trans"))
                for l in (dline.strip()).split()[-1:]:
                    to_highlight.append(("env", "goals", int(l)))
            elif "Environment initial condition inconsistent with transition relation" in dline:
                to_highlight.append(("env", "init"))
                to_highlight.append(("env", "trans"))


            # System unrealizability
            elif "System is unrealizable because the environment can force a safety violation" in dline:
                to_highlight.append(("sys", "trans"))
                to_highlight.append(("sys", "init"))
            elif "System highlighted goal(s) unrealizable" in dline:
                to_highlight.append(("sys", "trans"))
                to_highlight.append(("sys", "init"))
                for l in (dline.strip()).split()[-1:]:
                    to_highlight.append(("sys", "goals", int(l)))

            # Environment unrealizability
            elif "Environment is unrealizable because the system can force a safety violation" in dline:
                to_highlight.append(("env", "trans"))
            elif "Environment highlighted goal(s) unrealizable" in dline:
                to_highlight.append(("env", "trans"))
                for l in (dline.strip()).split()[-1:]:
                    to_highlight.append(("env", "goals", int(l)))

            if "unsatisfiable" in dline or "inconsistent" in dline :
                unsat = True

        subp.stdout.close()



        return (realizable, unsat, nonTrivial, to_highlight, output)



    def _coreFinding(self, to_highlight, unsat, badInit):
        #returns list of formulas that cause unsatisfiability/unrealizability (based on unsat flag).
        #takes as input sentences marked for highlighting, and formula describing bad initial states
        #from JTLV.

        #find number of states in automaton/counter for unsat/unreal core max unrolling depth ("recurrence diameter")
        proj_copy = deepcopy(self.proj)
        proj_copy.rfi = self.parser.proj.rfi
        proj_copy.sensor_handler = None
        proj_copy.actuator_handler = None
        proj_copy.h_instance = None

        num_bits = int(numpy.ceil(numpy.log2(len(self.parser.proj.rfi.regions))))  # Number of bits necessary to encode all regions
        region_props = ["bit" + str(n) for n in xrange(num_bits)]

        aut = fsa.Automaton(proj_copy)
        aut.loadFile(self.proj.getFilenamePrefix()+".aut", self.proj.enabled_actuators + self.proj.all_customs + region_props, self.proj.enabled_sensors, [])


        #find deadlocked states in the automaton (states with no out-transitions)
        deadStates = [s for s in aut.states if not s.transitions]
        #find states that can be forced by the environment into the deadlocked set
        forceDeadStates = [(s, e) for s in aut.states for e in deadStates if e in s.transitions]
        #LTL representation of these states and the deadlock-causing environment move in the next time step
        forceDeadlockLTL = map(lambda (s,e): " & ".join([stateToLTL(s), stateToLTL(e, 1, 1, True)]), forceDeadStates)


        #find livelocked goal and corresponding one-step propositional formula (by stripping LTL operators)
        desiredGoal = [h_item[2] for h_item in to_highlight if h_item[1] == "goals"]


        if desiredGoal:
            desiredGoal = desiredGoal[0]
            #Don't actually need LTL
            #desiredGoalLTL = stripLTLLine(self.ltlConjunctsFromBadLines([h_item for h_item in to_highlight if h_item[1] == "goals"], False)[0],True)



        def preventsDesiredGoal(s):
                rank_str = s.transitions[0].rank
                m = re.search(r"\(\d+,(-?\d+)\)", rank_str)
                if m is None:
                    logging.error("Error parsing jx in automaton.  Are you sure the spec is unrealizable?")
                    return
                jx = int(m.group(1))
                return (jx == desiredGoal)


        #find livelocked states in the automaton (states with desired sys rank)
        livelockedStates = filter(preventsDesiredGoal, [s for s in aut.states if s.transitions])
        #find states that can be forced by the environment into the livelocked set
        forceLivelockedStates = [(fro, to) for fro in aut.states for to in livelockedStates if to in s.transitions]

        #LTL representation of these states and the livelocked goal
        #forceLivelockLTL = map(lambda s: " & ".join([stateToLTL(s), desiredGoalLTL]), livelockedStates) ###Don't actually need to add goal -- will be added in 'conjuncts'
        forceLivelockLTL = map(lambda (s1,s2): " & ".join([stateToLTL(s1, 1, 1), stateToLTL(s2, 1, 0, True)]), forceLivelockedStates)
        #forceLivelockLTL = map(stateToLTL, livelockedStates)

        numStates = len(aut.states)
        numRegions = len(self.parser.proj.rfi.regions)

        if forceDeadlockLTL:
            deadlockFlag = True
            badStatesLTL = forceDeadlockLTL
        else:
            #this means livelock
            deadlockFlag = False
            badStatesLTL = forceLivelockLTL

        #################################
        #                               #
        # get conjuncts to be minimized #
        #                               #
        #################################

        #topology
        topo =self.spec['Topo'].replace('\n','')
        topo = topo.replace('\t','')

        #have to use all initial conditions if no single bad initial state given
        useInitFlag = badInit is None

        #other highlighted LTL formulas
        conjuncts = self.ltlConjunctsFromBadLines(to_highlight, useInitFlag)

        #filter out props that are actually used
        #self.propList = [p for p in self.propList if [c for c in conjuncts if p in c] or [c for c in badStatesLTL if p in c and not unsat] or p in topo]

        cmd = self._getPicosatCommand()

        if unsat:
            guilty = self.unsatCores(cmd, topo,badInit,conjuncts,15,15)#returns LTL
        else:
            guilty = self.unrealCores(cmd, topo, badStatesLTL, conjuncts, deadlockFlag)#returns LTL
        return guilty




    def unsatCores(self, cmd, topo, badInit, conjuncts,maxDepth,numRegions):
        #returns list of guilty LTL formulas
        #takes LTL formulas for topo, badInit and conjuncts separately because they are used in various combinations later
        #numStates and numRegions are used to determine unroll depth later

        if not conjuncts and badInit == "":
            #this means that the topology is unsatisfiable by itself (not common since we auto-generate)
            return topo
        else:
            #try the different cases of unsatisfiability (need to pass in command and proplist to coreUtils function)
            self.trans, guilty = unsatCoreCases(cmd, self.propList, topo, badInit, conjuncts,maxDepth,numRegions)

        return guilty



    def unrealCores(self, cmd, topo, badStatesLTL, conjuncts, deadlockFlag):
        #returns list of guilty LTL formulas FOR THE UNREALIZABLE CASE
        #takes LTL formulas representing the topology and other highlighted conjuncts as in the unsat case.
        #also takes a list of deadlocked/livelocked states (as LTL/propositional formulas)
        #returns LTL formulas that appear in the guilty set for *any* deadlocked or livelocked state,
        #i.e. formulas that cause deadlock/livelock in these states

        #try the different cases of unsatisfiability (need to pass in command and proplist to coreUtils function)
        if deadlockFlag:
            initDepth = 1
            maxDepth = 1
        else:
            initDepth = 1
            maxDepth = 1

#        TODO: see if there is a way to call pool.map with processes that also use pools
#
#        sys.stdout = StringIO.StringIO()
#
#        pool = Pool()
#        guiltyList = pool.map(unsatCoreCasesWrapper, itertools.izip(itertools.repeat(cmd), itertools.repeat(self.propList), itertools.repeat(topo), badStatesLTL, itertools.repeat(conjuncts), itertools.repeat(initDepth), itertools.repeat(maxDepth)))
#        pool.terminate()
#
#        sys.stdout = sys.__stdout__

        guiltyList = map(lambda d: unsatCoreCases(cmd, self.propList, topo, d, conjuncts, initDepth, maxDepth), badStatesLTL)

        guilty = reduce(set.union,map(set,[g for t, g in guiltyList]))

        return guilty





    def _getPicosatCommand(self):
        # look for picosat

        paths = [p for p in glob.glob(os.path.join(self.proj.ltlmop_root,"lib","cores","picosat-*")) if os.path.isdir(p)]
        if len(paths) == 0:
            logging.error("Where is your sat solver? We use Picosat.")
            # TODO: automatically compile for the user
            return None
        else:
            logging.debug("Found Picosat in " + paths[0])

        if os.name == "nt":
            cmd = os.path.join(paths[0],"picomus.exe")
        else:
            cmd = [os.path.join(paths[0],"picomus")]

        return cmd





    def ltlConjunctsFromBadLines(self, to_highlight, useInitFlag):
        #given the lines to be highlighted by the initial analysis, returns
        #a list of LTL formulas that, when conjuncted, cause unsatisfiability
        #topology conjuncts are separated out

        conjuncts = []

        for h_item in to_highlight:
            tb_key = h_item[0].title() + h_item[1].title()

            newCs = []
            if h_item[1] == "goals":
                #special treatment for goals: (1) we already know which one to highlight, and (2) we need to check both tenses
                #TODO: separate out the check for present and future tense -- what if you have to toggle but can still do so infinitely often?
                #newCs = ivd[self.traceback[tb_key][h_item[2]]].split('\n')
                goals = self.spec[tb_key].split('\n')
                newCs = [goals[h_item[2]]]
                newCsOld = newCs

            elif h_item[1] == "trans" or h_item[1] == "init" and useInitFlag:
                newCs =  self.spec[tb_key].replace("\t", "\n").split("\n")

            conjuncts.extend(newCs)

        return conjuncts

    def _synthesize(self):
        """ Call the synthesis tool, and block until it completes.
            Returns success flags `realizable` and `realizableFS`, and the raw
            synthesizer log output. """

        log_string = StringIO.StringIO()

        self._synthesizeAsync(log_function=log_string.write)

        self.synthesis_complete.wait()  # Block here until synthesis is done

        return (self.realizable, self.realizableFS, log_string.getvalue())

    def prepareSlugsInput(self):
        """ Convert from JTLV input format (.smv+.ltl) to Slugs input format (.slugsin)
            using the script provided by Slugs.

            This is a stop-gap fix; eventually we should just produce the input
            directly instead of using the conversion script. """

        # Add the conversion script to our path
        slugs_converter_path = os.path.join(self.proj.ltlmop_root, "etc", "slugs", "tools")
        sys.path.insert(0, slugs_converter_path)
        from translateFromLTLMopLTLFormatToSlugsFormat import performConversion

        # Call the conversion script
        with open(self.proj.getFilenamePrefix() + ".slugsin", "w") as f:
            # TODO: update performConversion so we don't have to do stdout redirection
            sys.stdout = f
            performConversion(self.proj.getFilenamePrefix() + ".smv", self.proj.getFilenamePrefix() + ".ltl")
            sys.stdout = sys.__stdout__

    def _synthesizeAsync(self, log_function=None, completion_callback_function=None):
        """ Asynchronously call the synthesis tool.  This function will return immediately after
            spawning a subprocess.  `log_function` will be called with a string argument every time
            the subprocess generates a line of text.  `completion_callback_function` will be called
            when synthesis finishes, with two arguments: the success flags `realizable`
            and `realizableFS`. """

        if self.proj.compile_options["synthesizer"].lower() == "jtlv":
            # Find the synthesis tool
            cmd = self._getGROneCommand("GROneMain")

            # Add any extra compiler options
            if self.proj.compile_options["fastslow"]:
                cmd.append("--fastslow")
            if self.proj.compile_options["symbolic"]:
                cmd.append("--symbolic")

            REALIZABLE_MESSAGE = "Specification is synthesizable!"
            REALIZABLE_FS_MESSAGE = "Specification is synthesizable under fast/slow!"

        elif self.proj.compile_options["synthesizer"].lower() == "slugs":
            # Find the synthesis tool
            cmd = self._getSlugsCommand()

            # Make sure flags are compatible
            if self.proj.compile_options["fastslow"]:
                raise RuntimeError("Slugs does not currently support fast/slow option.")
            if self.proj.compile_options["symbolic"]:
                cmd.extend(["--symbolicStrategy", self.proj.getFilenamePrefix() + ".bdd"])
            else:
                cmd.append(self.proj.getFilenamePrefix() + ".aut")

            # Create proper input for Slugs
            logging.info("Preparing Slugs input...")
            self.prepareSlugsInput()

            REALIZABLE_MESSAGE = "RESULT: Specification is realizable"
            REALIZABLE_FS_MESSAGE = None
        else:
            raise RuntimeError("Invalid synthesizer: {!r}".format(self.proj.compile_options["synthesizer"]))

        self.realizable = False
        self.realizableFS = False

        # Define some wrappers around the callback functions so we can parse the output
        # of the synthesis tool and return it in a meaningful way.
        def onLog(text):
            """ Intercept log callbacks to check for realizability status. """

            if REALIZABLE_MESSAGE in text:
                self.realizable = True
            if REALIZABLE_FS_MESSAGE is not None and REALIZABLE_FS_MESSAGE in text:
                self.realizableFS = True

            # You'll pass this on, won't you
            if log_function is not None:
                log_function(text)

        # Create a flag for convenience
        self.synthesis_complete = threading.Event()

        def onSubprocessComplete():
            if completion_callback_function is not None:
                completion_callback_function(self.realizable, self.realizableFS)
            self.synthesis_complete.set()
            self.synthesis_subprocess = None

        # Kick off the subprocess
        logging.info("Synthesizing a strategy...")

        self.synthesis_subprocess = AsynchronousProcessThread(cmd, onSubprocessComplete, onLog)

    def abortSynthesis(self):
        """ Kill any running synthesis process. """

        if self.synthesis_subprocess is not None:
            logging.warning("Aborting synthesis!")
            self.synthesis_subprocess.kill()
            self.synthesis_complete = None
            self.synthesis_subprocess = None

    def compile(self):
        if self.proj.compile_options["decompose"]:
            logging.info("Decomposing...")
            self._decompose()
        logging.info("Writing LTL file...")
        spec, tb, resp = self._writeLTLFile()
        logging.info("Writing SMV file...")
        self._writeSMVFile()

        if tb is None:
            logging.error("Compilation aborted")
            return

        #self._checkForEmptyGaits()

        return self._synthesize()

