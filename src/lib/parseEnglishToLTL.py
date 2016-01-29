""" 
    ===========================================================
    parseEnglishToLTL.py - Structured English to LTL Translator 
    ===========================================================
    
    Module that parses a set of structured English sentences into the
    corresponding LTL subformulas.
"""
import re
import copy
import numpy
import math
import logging

#nextify = lambda x: " next(%s) " % x

def nextify(p):
    # Recursive function for aggressively applying the next operator

    r_parens = re.compile('^\s*\((?P<inside>.*)\)\s*$',re.IGNORECASE)
    r_logic = re.compile('^(?P<left>.*) (?P<op>(or|and|\||\&)) (?P<right>.*)$',re.IGNORECASE)
    m_parens = r_parens.search(p)
    m_logic = r_logic.search(p)

    if m_parens:
        i = m_parens.group('inside') 
        return "( %s )" % nextify(i)
    elif m_logic:
        l = m_logic.group('left')
        r = m_logic.group('right')
        op = m_logic.group('op')
        return "%s %s %s" % (nextify(l), op, nextify(r))
    else:
        return " next(%s) " % p

def writeSpec(text, sensorList, regionList, actuatorList, customsList, fastslow=False, use_bits=True):
    ''' This function creates the Spec dictionary that contains the parsed LTL
        subformulas. It takes the text that contains the structured English,
        the list of sensor propositions, the list containing
        the region names and the list of robot propositions (other than regions). 
    '''

    failed = False

    regionList = copy.deepcopy(regionList)
    actuatorList = copy.deepcopy(actuatorList)
    customsList = copy.deepcopy(customsList)
    sensorList = copy.deepcopy(sensorList)

    # Prepend "e." or "s." to propositions for JTLV
    for i, sensor in enumerate(sensorList):
        text = re.sub("\\b"+sensor+"\\b", "e." + sensor, text)
        sensorList[i] = "e." + sensorList[i]

    for i, prop in enumerate(actuatorList):
        text = re.sub("\\b"+prop+"\\b", "s." + prop, text)
        actuatorList[i] = "s." + actuatorList[i]

    for i, prop in enumerate(customsList):
        text = re.sub("\\b"+prop+"\\b", "s." + prop, text)
        customsList[i] = "s." + customsList[i]

    # initializing the dictionary
    spec = {}
    spec['EnvInit']= ''
    spec['EnvTrans']= ''
    spec['EnvGoals']= ''
    spec['SysInit']= ''
    spec['SysTrans']= ''
    spec['SysGoals']= ''


    linemap = {}
    linemap['EnvInit']= []
    linemap['EnvTrans']= []
    linemap['EnvGoals']= []
    linemap['SysInit']= []    
    linemap['SysTrans']= []
    linemap['SysGoals']= []
    
    LTL2LineNo = {}

    RegionGroups = {}

    # list of robotProp (with actuators and customs propositions)
    robotPropList = copy.deepcopy(actuatorList + customsList)

    # List of all robot prpositions
    allRobotProp = regionList + robotPropList

    # Define the number of bits needed to encode the regions
    numBits = int(numpy.ceil(numpy.log2(len(regionList))))

    # creating the region bit encoding
    bitEncode = bitEncoding(len(regionList),numBits)
    currBitEnc = bitEncode['current']
    nextBitEnc = bitEncode['next']
    envBitEnc = bitEncode['env']
    envNextBitEnc = bitEncode['envNext']



    # Regular expressions to help us out
    EnvInitRE = re.compile('^(environment|env) starts with',re.IGNORECASE)
    SysInitRE = re.compile('^(robot |you |)starts?',re.IGNORECASE)
    IfThenRE = re.compile('if (?P<cond>.+) then (?P<req>.+)',re.IGNORECASE)
    UnlessRE = re.compile('(?P<req>.+) unless (?P<cond>.+)',re.IGNORECASE)
    IffRE = re.compile('(?P<req>.+) if and only if (?P<cond>.+)',re.IGNORECASE)
    CommentRE = re.compile('^\s*#',re.IGNORECASE)
    LivenessRE = re.compile('^\s*(go to|visit|infinitely often do|infinitely often sense|infinitely often)',re.IGNORECASE)
    if fastslow:
        SafetyRE = re.compile('^\s*(always|always do |do|always sense|sense|finished|finish|have)',re.IGNORECASE)
        SafetyCompletionRE = re.compile('^\s*(finished|not finished|have not finished|have finished|do not finish|finish|not finish)',re.IGNORECASE)
    else:
        SafetyRE = re.compile('^\s*(always|always do |do|always sense|sense)',re.IGNORECASE)
    StayRE = re.compile('(stay there|stay)',re.IGNORECASE)
    AtLeastOnceRE = re.compile('(at least once)',re.IGNORECASE)
    AfterEachTimeRE = re.compile('after each time (?P<cond>.+),(?P<req>.+)(?:at least once)?',re.IGNORECASE)
    EventRE = re.compile('(?P<prop>[\w\.]+) is set on (?P<setEvent>.+) and reset on (?P<resetEvent>.+)',re.IGNORECASE)
    ToggleRE = re.compile('(?P<prop>[\w\.]+) is toggled (when|on) (?P<toggleEvent>.+)',re.IGNORECASE)
    RegionGroupingRE = re.compile('group (?P<groupName>[\w]+) (is|are) (?P<regions>.+)',re.IGNORECASE)
    QuantifierRE = re.compile('\\b(?P<quantifier>all|any)\s+(?P<groupName>\w+)',re.IGNORECASE)



    internal_props = []
    # Creating the 'Stay' formula - it is a constant formula given the number of bits.
    StayFormula = createStayFormula(regionList, use_bits=use_bits, fastslow=fastslow)

    lineInd = 0
    


    # iterate over the lines in the file
    for line in text.split("\n"):

        lineInd = lineInd + 1

        # If it is an empty line, ignore it
        if re.search('^(\s*)$',line):
            continue
            
        # If the sentence is a comment, ignore it
        if CommentRE.search(line):
            continue

        # If there is a newline at the end, remove it
        if re.search('\n$',line):
            line = re.sub('\n$','',line)
        # If there are ',' or '.', remove then
        line = line.replace('\B.\B',' ') # Leave periods that are in the middle of words.

        # remove commas except for in region group definitions which need them for the list
        # and AET statements which use it as a delimiter
        if not (RegionGroupingRE.search(line) or AfterEachTimeRE.search(line)):
            line = line.replace(',',' ')

        # Check for the presence of a region quantifier
        if QuantifierRE.search(line):
            QuantifierParts = QuantifierRE.search(line)
            quant_type = QuantifierParts.group('quantifier') 
            quant_group = QuantifierParts.group('groupName') 
            #print "found quantifier '%s' for group '%s'" % (quant_type, quant_group)
            if quant_group not in RegionGroups:
                print 'ERROR(1): Could not parse the sentence in line '+ str(lineInd)+' :'
                print line
                print 'because no region grouping is previously defined for group name: ' + quant_group
                failed = True
                continue

            line = QuantifierRE.sub('QUANTIFIER_PLACEHOLDER',line)
            QuantifierFlag = quant_type.upper()
            allRobotProp.append("QUANTIFIER_PLACEHOLDER")
            customsList.append("QUANTIFIER_PLACEHOLDER")

            quant_or_string = {}
            quant_and_string = {}
            if len(RegionGroups[quant_group]) > 0:
                quant_and_string['current'] = "(" + " & ".join(RegionGroups[quant_group]) + ")"
                quant_and_string['next'] = "(" + " & ".join(map(nextify, RegionGroups[quant_group])) + ")"
                quant_or_string['current'] = "(" + " | ".join(RegionGroups[quant_group]) + ")"
                quant_or_string['next'] = "(" + " | ".join(map(nextify, RegionGroups[quant_group])) + ")"
            else:
                # With an empty group, it's impossible to be in any or all of them
                quant_and_string['current'] = "FALSE"
                quant_and_string['next'] = "FALSE"
                quant_or_string['current'] = "FALSE"
                quant_or_string['next'] = "FALSE"

            #quant_or_string['current'] = replaceLogicOp(quant_or_string['current'])
            #quant_and_string['current'] = replaceLogicOp(quant_and_string['current'])
            #quant_or_string['next'] = replaceLogicOp(quant_or_string['next'])
            #quant_and_string['next'] = replaceLogicOp(quant_and_string['next'])
        else:
            QuantifierFlag = None

        # If the sentence defines a group of regions
        if RegionGroupingRE.search(line):
            RGParts = RegionGroupingRE.search(line)
            groupName = RGParts.group('groupName') 
            groupList = RGParts.group('regions') 
            RegionGroups[groupName] = re.split(r"\s*,\s*", groupList)
            RegionGroups[groupName] = map(replaceLogicOp, RegionGroups[groupName])

            # Replace with completion props if we are using fastslow
            if fastslow:
                newRegionGroup = []
                for formula in RegionGroups[groupName]:
                    for prop in regionList:
                        formula = re.sub('(?<=[! &|(\t\n])'+prop+'(?=[ &|)\t\n])', 'e.'+prop.replace('s.','')+'_rc', formula)
                    newRegionGroup.append(formula)
                RegionGroups[groupName] = newRegionGroup

            # 'empty' is a no-op 
            if 'empty' in RegionGroups[groupName]:
                RegionGroups[groupName].remove('empty')

            # Allow equivalency between basic singular/plural references
            if groupName[-1] == "s":
                RegionGroups[groupName[0:-1]] = RegionGroups[groupName]

            #print RegionGroups

        # If the sentence describes the initial state of the environemnt
        elif EnvInitRE.search(line):

            # remove the first words     
            EnvInit = EnvInitRE.sub('',line)

            if 'finished ' in EnvInit:
                print 'ERROR(18): Could not parse the sentence in line '+ str(lineInd)+' :'
                print line
                print 'because completion propositions are not valid in environment initial conditions'
                failed = True
                continue

            # parse the rest and return it to spec['EnvInit']
            if len(sensorList) == 0:
                LTLsubformula = ''
            else:
                LTLsubformula = parseInit(EnvInit,sensorList,lineInd)
                if LTLsubformula == '': failed = True

            # this shouldn't even be possible, but check just in case:
            if "QUANTIFIER_PLACEHOLDER" in LTLsubformula:
                print 'ERROR(15): Could not parse the sentence in line '+ str(lineInd)+' :'
                print line
                print 'because quantifiers are not valid in environment initial conditions'
                failed = True
                continue

            ### put parentheses around the env init condition for DNF later #####
            LTLsubformula = LTLsubformula[0:LTLsubformula.rfind('&')]
            spec['EnvInit']= '&\n'.join(filter(None, [spec['EnvInit'], "(" + LTLsubformula + ")"]))
            linemap['EnvInit'].append(lineInd)
            
            LTL2LineNo[replaceRegionName(LTLsubformula,bitEncode,regionList)] = lineInd
            
        # If the sentence describes the initial state of the robot
        elif SysInitRE.search(line):
            # remove the first words     
            SysInit = SysInitRE.sub('',line)

            RegInit = ''
            ActInit = ''
            LTLRegSubformula = ''
            LTLActSubformula = ''
            LTLEnvRegSubformula = ''
            LTLEnvActSubformula = ''
            
            # divide, if needed, into region initial condition and action
            # (other propositions) initial conditions  
            if re.search('in (.+) with (.+)',SysInit):
                RegInit = re.search('in (.+) with (.+)',SysInit).groups()[0]
                ActInit = re.search('in (.+) with (.+)',SysInit).groups()[1]

            # Only region initial condition    
            elif ' in ' in SysInit:
                RegInit = SysInit.replace(' in ', ' ')

            # Only action initial condition    
            elif ' with ' in SysInit:
                ActInit = SysInit.replace(' with ', ' ')

            if RegInit:
                # parse regions
                LTLRegSubformula = parseInit(RegInit,regionList + ["QUANTIFIER_PLACEHOLDER"],lineInd)
                if LTLRegSubformula == '': failed = True

                # we are going there and we are in there
                LTLEnvRegSubformula = LTLRegSubformula
                if fastslow:
                    for prop in regionList:
                        LTLEnvRegSubformula = re.sub('(?<=[! &|(\t\n])'+prop+'(?=[ &|)\t\n])', 'e.'+prop.replace('s.','')+'_rc', LTLEnvRegSubformula)
            if ActInit:
                if len(robotPropList) == 0:
                    LTLActSubformula = ''
                else:
                    # parse Actions
                    LTLActSubformula = parseInit(ActInit,robotPropList,lineInd)
                    if LTLActSubformula == '': failed = True
                    # we are trying to activate the action and we are doing it
                    LTLEnvActSubformula = LTLActSubformula
                    if fastslow:
                        for prop in actuatorList:
                            LTLEnvActSubformula = re.sub('(?<=[! &|(\t\n])'+prop+'(?=[ &|)\t\n])', 'e.'+prop.replace('s.','')+'_ac', LTLEnvActSubformula)

            if QuantifierFlag == "ANY":
                LTLRegSubformula = LTLRegSubformula.replace("QUANTIFIER_PLACEHOLDER", quant_or_string['current'])
            elif QuantifierFlag == "ALL":
                LTLRegSubformula = LTLRegSubformula.replace("QUANTIFIER_PLACEHOLDER", quant_and_string['current'])

            ### put parentheses around the sys init condition for DNF later #####
            spec['SysInit']= '& '.join(filter(None,[x.strip().rstrip('&') for x in [spec['SysInit'], LTLRegSubformula, LTLActSubformula]]))

            linemap['SysInit'].append(lineInd)            
            LTL2LineNo[replaceRegionName(LTLRegSubformula + LTLActSubformula,bitEncode,regionList)] = lineInd    
            # also add to envInit if we are running instantaneous action
            if fastslow:
                spec['EnvInit']= '& '.join(filter(None, [x.strip().rstrip('&') for x in [spec['EnvInit'], LTLEnvRegSubformula, LTLEnvActSubformula]]))
                linemap['EnvInit'].append(lineInd)
                LTL2LineNo[replaceRegionName(LTLEnvRegSubformula + LTLEnvActSubformula,bitEncode,regionList)] = lineInd

        # If the sentence is a conditional 
        elif IfThenRE.search(line) or UnlessRE.search(line) or IffRE.search(line) :

            if IfThenRE.search(line):
                CondParts = IfThenRE.search(line)
                CondType = 'IfThen'
            elif UnlessRE.search(line):
                CondParts = UnlessRE.search(line)
                CondType = 'Unless'
            else :
                CondParts = IffRE.search(line)
                CondType = 'IFF'

            # Extract the 2 pieces (condition and requirement)  
            Condition = CondParts.group('cond')
            Requirement = CondParts.group('req')
            # Replace any quantifier in the condition clause
            if "QUANTIFIER_PLACEHOLDER" in Condition:
                if QuantifierFlag == "ANY":
                    Condition = Condition.replace("next(QUANTIFIER_PLACEHOLDER)", quant_or_string['next'])
                    Condition = Condition.replace("QUANTIFIER_PLACEHOLDER", quant_or_string['current'])
                elif QuantifierFlag == "ALL":
                    Condition = Condition.replace("next(QUANTIFIER_PLACEHOLDER)", quant_and_string['next'])
                    Condition = Condition.replace("QUANTIFIER_PLACEHOLDER", quant_and_string['current'])

                # Since we are only supporting one quantifier per line, skip the requirement parsing below
                QuantifierFlag = None

            # Figure out what the requirement is and parse it
            if LivenessRE.search(Requirement):

                # remove first words
                Requirement = LivenessRE.sub(' ',Requirement)

                # check for "stay there" condition
                if StayRE.search(Requirement):
                    # remove the 'and stay there'      
                    Requirement = Requirement.replace(' and stay there','')

                    # parse the liveness requirement
                    ReqFormulaInfo = parseLiveness(Requirement,sensorList,regionList,actuatorList,customsList,lineInd,fastslow)
                    if ReqFormulaInfo['formula'] == '': failed = True

                    # If not SysGoals, then it is an error
                    if not ReqFormulaInfo['type']=='SysGoals':
                        print 'ERROR(15): Could not parse the sentence in line '+ str(lineInd)+' :'
                        print line
                        print 'because the requirement is not system liveness'
                        failed = True
                        continue

                    if CondType == "IFF":
                        print 'ERROR(15): Could not parse the sentence in line '+ str(lineInd)+' :'
                        print line
                        print 'because IFF cannot be used with "stay there"'
                        failed = True
                        continue

                    # add the 'stay there' as a condition (if R then stay there)
                    regCond = ReqFormulaInfo['formula'].replace('\t\t\t []<>','')
                    regCond = regCond.replace('& \n','')

                    # Replace any quantifier in the regCond clause
                    if QuantifierFlag == "ANY":
                        regCond = regCond.replace("QUANTIFIER_PLACEHOLDER", quant_or_string['current'])
                    elif QuantifierFlag == "ALL":
                        regCond = regCond.replace("QUANTIFIER_PLACEHOLDER", quant_and_string['current'])

                    condStayFormula = {}
                    condStayFormula['formula'] = '\t\t\t [](' + regCond + ' -> ' + StayFormula + ') & \n'
                    condStayFormula['type'] = 'SysGoals'  # HACK: this is obviously SysTrans, but we don't want to allow any next()s 

                    # Parse the condition and add it to the requirement
                    CondFormulaInfo = parseConditional(Condition,condStayFormula,CondType,sensorList,regionList,actuatorList,customsList,lineInd,fastslow)
                    CondFormulaInfo['type'] = 'SysTrans'
                    if CondFormulaInfo['formula'] == '': failed = True

                    spec[CondFormulaInfo['type']] = spec[CondFormulaInfo['type']] + CondFormulaInfo['formula']
                    linemap[CondFormulaInfo['type']].append(lineInd)
                    LTL2LineNo[replaceRegionName(CondFormulaInfo['formula'],bitEncode,regionList)] = lineInd

                # check for "at least once" condition
                elif AtLeastOnceRE.search(Requirement):
                    # remove the 'at least once'      
                    Requirement = Requirement.replace(' at least once','')

                    # parse the liveness requirement
                    ReqFormulaInfo = parseLiveness(Requirement,sensorList,regionList,actuatorList,customsList,lineInd,fastslow)
                    if ReqFormulaInfo['formula'] == '': failed = True

                    # If not SysGoals, then it is an error
                    if not ReqFormulaInfo['type']=='SysGoals':
                        print 'ERROR(15): Could not parse the sentence in line '+ str(lineInd)+' :'
                        print line
                        print 'because the requirement is not system liveness'
                        failed = True
                        continue

                    if CondType == "IFF":
                        print 'ERROR(15): Could not parse the sentence in line '+ str(lineInd)+' :'
                        print line
                        print 'because IFF cannot be used with "at least once"'
                        failed = True
                        continue

                    ### example ###
                    # "if s then visit r1 and a at least once"
                    # ... becomes ...
                    # []<>(s->m_r1_a)
                    # [](next(m_r1_a) <-> (m_r1_a | (next(r1) & next(a))))
                    
                    regCond = ReqFormulaInfo['formula'].replace('\t\t\t []<>','')
                    regCond = regCond.replace('& \n','')

                    if QuantifierFlag == "ANY":
                        print "not implemented yet"
                        failed = True
                    elif QuantifierFlag == "ALL":
                        iterate_over = RegionGroups[quant_group]
                    else:
                        iterate_over = ["total hack"]

                    memPropNames = []

                    for r in iterate_over:
                        tmp_req = regCond.replace("next(QUANTIFIER_PLACEHOLDER)", nextify(r))
                        tmp_req = tmp_req.replace("QUANTIFIER_PLACEHOLDER", r)
                        
                        internal_props.append("m" + re.sub("(e|s)\.", "", re.sub("\s+","_",tmp_req.replace("&","").replace("(","").replace(")",""))).rstrip("_"))
                        memPropNames.append("s."+internal_props[-1])

                        condStayFormula = {}
                        condStayFormula['formula'] = '\t\t\t [](next({0}) <-> ({0} | ({1}))) & \n'.format(memPropNames[-1], nextify(tmp_req))
                        condStayFormula['type'] = 'SysTrans'

                        spec[condStayFormula['type']] = spec[condStayFormula['type']] + condStayFormula['formula']
                        linemap[condStayFormula['type']].append(lineInd)
                        LTL2LineNo[replaceRegionName(condStayFormula['formula'],bitEncode,regionList)] = lineInd

                    ReqFormulaInfo['formula'] = '\t\t\t []<>(' + ' & '.join(memPropNames) + ') & \n'
                else:
                    # parse requirement normally
                    ReqFormulaInfo = parseLiveness(Requirement,sensorList,regionList,actuatorList,customsList,lineInd,fastslow)
                    if ReqFormulaInfo['formula'] == '': failed = True
            elif SafetyRE.search(Requirement):
                if fastslow:
                    #check if we are using the 'finished' keyword
                    if SafetyCompletionRE.search(Requirement):
                        CompletionFlag = True
                    else:
                        CompletionFlag = False

                # remove first words
                Requirement = SafetyRE.sub(' ',Requirement)

                # and parse requirement
                if fastslow:
                    ReqFormulaInfo = parseSafety(Requirement,sensorList,regionList,actuatorList,customsList,lineInd,CompletionFlag)
                else:
                    ReqFormulaInfo = parseSafety(Requirement,sensorList,regionList,actuatorList,customsList,lineInd)
                if ReqFormulaInfo['formula'] == '': failed = True

            elif StayRE.search(Requirement):
                ReqFormulaInfo = {}
                # The formula - adding the '\t\t\t []' so it will be added to the conditional
                # ( a bit of a hack, but it is just to make it a 'safety' requirement)
                ReqFormulaInfo['formula'] = '\t\t\t []' + StayFormula
                # the type - it has to be the system safety requirement
                ReqFormulaInfo['type'] = 'SysTrans'
                
            else:
                print 'ERROR(13): Could not parse the sentence in line '+ str(lineInd)+' :'
                print line
                print 'because could not resolve the requirement:'
                print Requirement
                print ''
                failed = True
                continue

            # Replace any quantifier in the requirement
            if QuantifierFlag == "ANY":
                if 'Goals' in ReqFormulaInfo['type']:   # liveness
                    ReqFormulaInfo['formula'] = ReqFormulaInfo['formula'].replace("QUANTIFIER_PLACEHOLDER", quant_or_string['current'])
                elif 'Trans' in ReqFormulaInfo['type']:   # safety
                    print 'ERROR(13): Could not parse the sentence in line '+ str(lineInd)+' :'
                    print line
                    print 'because the ANY quantifier is not currently supported in safety requirements'
                    print ''
                    failed = True
                    continue

                # Parse the condition and add it to the requirement
                CondFormulaInfo = parseConditional(Condition,ReqFormulaInfo,CondType,sensorList,regionList,actuatorList,customsList,lineInd,fastslow)
                if CondFormulaInfo['formula'] == '': failed = True
                spec[CondFormulaInfo['type']] = spec[CondFormulaInfo['type']] + CondFormulaInfo['formula']
                linemap[CondFormulaInfo['type']].append(lineInd)
                LTL2LineNo[replaceRegionName(CondFormulaInfo['formula'],bitEncode,regionList)] = lineInd
            elif QuantifierFlag == "ALL":
                for r in RegionGroups[quant_group]:
                    tmp_req = copy.deepcopy(ReqFormulaInfo)
                    tmp_req['formula'] = tmp_req['formula'].replace("next(QUANTIFIER_PLACEHOLDER)", nextify(r))
                    tmp_req['formula'] = tmp_req['formula'].replace("QUANTIFIER_PLACEHOLDER", r)
                    # Parse the condition and add it to the requirement
                    CondFormulaInfo = parseConditional(Condition,tmp_req,CondType,sensorList,regionList,actuatorList,customsList,lineInd,fastslow)
                    if CondFormulaInfo['formula'] == '': failed = True
                    spec[CondFormulaInfo['type']] = spec[CondFormulaInfo['type']] + CondFormulaInfo['formula']
                    linemap[CondFormulaInfo['type']].append(lineInd)
                    LTL2LineNo[replaceRegionName(CondFormulaInfo['formula'],bitEncode,regionList)] = lineInd
            else:
                # Parse the condition and add it to the requirement
                CondFormulaInfo = parseConditional(Condition,ReqFormulaInfo,CondType,sensorList,regionList,actuatorList,customsList,lineInd,fastslow)
                if CondFormulaInfo['formula'] == '': failed = True
                spec[CondFormulaInfo['type']] = spec[CondFormulaInfo['type']] + CondFormulaInfo['formula']
                linemap[CondFormulaInfo['type']].append(lineInd)
                LTL2LineNo[replaceRegionName(CondFormulaInfo['formula'],bitEncode,regionList)] = lineInd
                
        # An "after each time" implicit memory statement
        elif AfterEachTimeRE.search(line):
            if fastslow:
                print 'ERROR(17): AFTER EACH TIME is not yet supported for instantaneous action, line '+ str(lineInd)+'\n'
                failed = True
                continue

            AfterEachTimeParts = AfterEachTimeRE.search(line) 

            # Extract the 2 pieces (condition and requirement)  
            Condition = AfterEachTimeParts.group('cond')
            Requirement = AfterEachTimeParts.group('req')
    
            if QuantifierFlag == "ANY" or QuantifierFlag == "ALL":
                print 'ERROR(6): Quantifiers not yet supported inside "After each time" statements, line '+ str(lineInd)+'\n'
                failed = True
                continue

            # Figure out what the requirement is and parse it
            # FIXME: This doesn't really fit nicely into the category of either liveness or safety;
            # it's semantically closest to a bare LTL <>
            if not LivenessRE.search(Requirement):
                print 'ERROR(13): Could not parse the sentence in line '+ str(lineInd)
                print 'because only livenesses are currently supported for "After each time"'
                failed = True
                continue

            # remove first words
            Requirement = LivenessRE.sub(' ',Requirement)

            AETFormula_Safety, AETFormula_Goal, mem_prop = parseAfterEachTime(Condition, Requirement, sensorList, regionList, robotPropList, lineInd, StayFormula)
            if AETFormula_Safety == '': failed = True

            spec["SysTrans"] += AETFormula_Safety
            linemap["SysTrans"].append(lineInd)
            LTL2LineNo[replaceRegionName(AETFormula_Safety,bitEncode,regionList)] = lineInd

            spec["SysGoals"] += AETFormula_Goal
            linemap["SysGoals"].append(lineInd)
            LTL2LineNo[replaceRegionName(AETFormula_Goal,bitEncode,regionList)] = lineInd

            internal_props.append(mem_prop)

        # An event definition
        elif EventRE.search(line):
            # get the groups
            EventParts = EventRE.search(line)
            EventProp = EventParts.group('prop') 
            SetEvent = EventParts.group('setEvent') 
            ResetEvent = EventParts.group('resetEvent') 


            # Formulas defining when the proposition is true and false
            EventFormula = parseEvent(EventProp,SetEvent,ResetEvent,sensorList,regionList,actuatorList,customsList,lineInd,fastslow)
            if EventFormula == '': failed = True

            # Replace any quantifier in the event formula
            if QuantifierFlag == "ANY":
                EventFormula = EventFormula.replace("next(QUANTIFIER_PLACEHOLDER)", quant_or_string['next'])
                EventFormula = EventFormula.replace("QUANTIFIER_PLACEHOLDER", quant_or_string['current'])
            elif QuantifierFlag == "ALL":
                EventFormula = EventFormula.replace("next(QUANTIFIER_PLACEHOLDER)", quant_and_string['next'])
                EventFormula = EventFormula.replace("QUANTIFIER_PLACEHOLDER", quant_and_string['current'])

            spec['SysTrans'] = spec['SysTrans'] + EventFormula
            linemap['SysTrans'].append(lineInd)
            LTL2LineNo[replaceRegionName(EventFormula,bitEncode,regionList)] = lineInd


        # A toggle event definition
        elif ToggleRE.search(line):

            # get the groups
            EventParts = ToggleRE.search(line)
            EventProp = EventParts.group('prop') 
            ToggleEvent = EventParts.group('toggleEvent') 

            # Formulas defining when the proposition is true and false
            EventFormula = parseToggle(EventProp,ToggleEvent,sensorList,regionList,actuatorList,customsList,lineInd,fastslow)
            if EventFormula == '': failed = True

            # Replace any quantifier in the event formula
            if QuantifierFlag == "ANY":
                EventFormula = EventFormula.replace("next(QUANTIFIER_PLACEHOLDER)", quant_or_string['next'])
                EventFormula = EventFormula.replace("QUANTIFIER_PLACEHOLDER", quant_or_string['current'])
            elif QuantifierFlag == "ALL":
                EventFormula = EventFormula.replace("next(QUANTIFIER_PLACEHOLDER)", quant_and_string['next'])
                EventFormula = EventFormula.replace("QUANTIFIER_PLACEHOLDER", quant_and_string['current'])

            spec['SysTrans'] = spec['SysTrans'] + EventFormula
            linemap['SysTrans'].append(lineInd)
            LTL2LineNo[replaceRegionName(EventFormula,bitEncode,regionList)] = lineInd

        # A 'Go to and stay there' requirement
        elif LivenessRE.search(line) and StayRE.search(line):
            #TODO: quantifier support
            # remove the liveness words    
            LivenessReq = LivenessRE.sub('',line)
            # remove the 'and stay there'      
            LivenessReq = LivenessReq.replace(' and stay there','')

            # parse the liveness requirement
            formulaInfo = parseLiveness(LivenessReq,sensorList,regionList,actuatorList,customsList,lineInd,fastslow)

            # If could not parse the liveness
            if formulaInfo['formula']=='':
                print 'ERROR(14): Could not parse the sentence in line '+ str(lineInd)+' :'
                print line
                print 'because could not parse liveness requirement'
                failed = True
                continue
                
            # If not SysGoals, then it is an error
            if not formulaInfo['type']=='SysGoals':
                print 'ERROR(15): Could not parse the sentence in line '+ str(lineInd)+' :'
                print line
                print 'because the requirement is not system liveness'
                failed = True
                continue

            # Add the liveness ('go to') to the spec
            spec[formulaInfo['type']] = spec[formulaInfo['type']] + formulaInfo['formula']
            linemap[formulaInfo['type']].append(lineInd)
            LTL2LineNo[replaceRegionName(formulaInfo['formula'],bitEncode,regionList)] = lineInd

            # add the 'stay there' as a condition (if R then stay there)
            regCond = formulaInfo['formula'].replace('\t\t\t []<>','')
            regCond = regCond.replace('& \n','')
            condStayFormula = '\t\t\t [](' + regCond + ' -> ' + StayFormula + ') & \n'

            spec['SysTrans'] = spec['SysTrans'] + condStayFormula
            linemap['SysTrans'].append(lineInd)
            LTL2LineNo[replaceRegionName(condStayFormula,bitEncode,regionList)] = lineInd



        # A liveness requirement
        elif LivenessRE.search(line):
            # remove the first words     
            LivenessReq = LivenessRE.sub('',line)

            # parse the liveness requirement
            formulaInfo = parseLiveness(LivenessReq,sensorList,regionList,actuatorList,customsList,lineInd,fastslow)
            if formulaInfo['formula'] == '': failed = True

            # Replace any quantifier in the requirement
            if QuantifierFlag == "ANY":
                formulaInfo['formula'] = formulaInfo['formula'].replace("QUANTIFIER_PLACEHOLDER", quant_or_string['current'])
                spec[formulaInfo['type']] = spec[formulaInfo['type']] + formulaInfo['formula']
                linemap[formulaInfo['type']].append(lineInd)
            elif QuantifierFlag == "ALL":
                for r in RegionGroups[quant_group]:
                    tmp_req = copy.deepcopy(formulaInfo)
                    tmp_req['formula'] = tmp_req['formula'].replace("QUANTIFIER_PLACEHOLDER", r)
                    spec[tmp_req['type']] = spec[tmp_req['type']] + tmp_req['formula']
                    linemap[tmp_req['type']].append(lineInd)
            else:
                spec[formulaInfo['type']] = spec[formulaInfo['type']] + formulaInfo['formula']
                linemap[formulaInfo['type']].append(lineInd)
           
            
            LTL2LineNo[replaceRegionName(formulaInfo['formula'],bitEncode,regionList)] = lineInd


        # A safety requirement
        elif SafetyRE.search(line):

            # remove the first words
            SafetyReq = SafetyRE.sub('',line)

            if fastslow:
                #check if we are using the 'finished' keyword
                if SafetyCompletionRE.search(SafetyReq):
                    CompletionFlag = True
                    SafetyReq = SafetyRE.sub('',SafetyReq)
                else:
                    CompletionFlag = False

            # parse the safety requirement
            if fastslow:
                formulaInfo = parseSafety(SafetyReq,sensorList,regionList,actuatorList,customsList,lineInd,CompletionFlag)
            else:
                formulaInfo = parseSafety(SafetyReq,sensorList,regionList,actuatorList,customsList,lineInd)
            if formulaInfo['formula'] == '': failed = True 

            # Replace any quantifier in the requirement
            if QuantifierFlag == "ANY":
                print 'ERROR(13): Could not parse the sentence in line '+ str(lineInd)+' :'
                print line
                print 'because the ANY quantifier is not currently supported in safety requirements'
                print ''
                failed = True
                continue
            elif QuantifierFlag == "ALL":
                for r in RegionGroups[quant_group]:
                    tmp_req = copy.deepcopy(formulaInfo)
                    tmp_req['formula'] = tmp_req['formula'].replace("next(QUANTIFIER_PLACEHOLDER)", nextify(r))
                    tmp_req['formula'] = tmp_req['formula'].replace("QUANTIFIER_PLACEHOLDER", r)
                    spec[tmp_req['type']] = spec[tmp_req['type']] + tmp_req['formula']
                    linemap[tmp_req['type']].append(lineInd)
            else:
                spec[formulaInfo['type']] = spec[formulaInfo['type']] + formulaInfo['formula']
                linemap[formulaInfo['type']].append(lineInd)
            
            LTL2LineNo[replaceRegionName(formulaInfo['formula'],bitEncode,regionList)] = lineInd


            
        # Cannot parse
        else:
            print 'ERROR(16): Could not parse the sentence in line '+ str(lineInd)+' :'
            print line
            print 'because it is not in a known format'
            print ''
            failed = True

        ## Resolve all the quantifiers
        #if QuantifierFlag is not None:
        #    for f_type in ['EnvInit', 'SysInit', 'EnvTrans', 'SysTrans', 'EnvGoals', 'SysGoals']:
        #        for i, f in enumerate(spec[f_type]):
        #            print "!!" + f
        #            if "QUANTIFIER_PLACEHOLDER" not in f: continue
        #            if f_type in ['EnvInit']:
        #                print "ERROR: Quantifier not valid in environment initial conditions"
        #            isCondition = "->" in f[f.find("QUANTIFIER_PLACEHOLDER"):]
        #            if QuantifierFlag == "ANY":
        #                spec[f_type][i] = f.replace("QUANTIFIER_PLACEHOLDER", or_string)
        #            elif QuantifierFlag == "ALL":
        #                if isCondition:
        #                    spec[f_type][i] = f.replace("QUANTIFIER_PLACEHOLDER", and_string)
        #                elif f_type in ["EnvTrans", "SysTrans"]:
        #                    for r in RegionGroups[quant_group]:
        #                        spec[f_type].append(f.replace("QUANTIFIER_PLACEHOLDER", r))
        #                    spec[f_type][i] = ""
        #                elif f_type in ["EnvGoals", "SysGoals"]:
        #                    for r in RegionGroups[quant_group]:
        #                        spec[f_type].append(f.replace("QUANTIFIER_PLACEHOLDER", r))
        #                    spec[f_type][i] = ""
        #    allRobotProp.remove("QUANTIFIER_PLACEHOLDER")
      
        if QuantifierFlag is not None:
            allRobotProp.remove("QUANTIFIER_PLACEHOLDER")
            customsList.remove("QUANTIFIER_PLACEHOLDER")

    # Setting all empty subformulas to TRUE, and removing last & in 'EnvGoals' and 'SysGoals'

    if spec['EnvInit'] == '':
        spec['EnvInit'] = '\t\t\tTRUE & \n'

    if spec['EnvTrans'] == '':
        if not fastslow or (fastslow and not regionList):
            spec['EnvTrans'] = '\t\t\t[](TRUE) & \n'

    if spec['EnvGoals'] == '':
        if not fastslow or (fastslow and not (regionList + actuatorList)):
            spec['EnvGoals'] = '\t\t\t[]<>(TRUE)'
        
    else:
        # remove last &
        spec['EnvGoals'] = re.sub('& \n$','\n',spec['EnvGoals'])

    # No need to change anything in SysTrans,
    # since the transition relation is encoded anyway
   
    if spec['SysGoals'] == '':
        spec['SysGoals'] = '\t\t\t[]<>(TRUE)'
    else:
        # remove last &
        spec['SysGoals'] = re.sub('& \n$','\n',spec['SysGoals'])



    # Check for unused sensor or robot propositions
    specstr = ''
    # put all Trans and Goal formulas into one string for ease of search
    for key in spec:
        if not 'Init' in key:
            specstr = specstr + spec[key]
    # search the string
    unusedProp = []
    for prop in sensorList + robotPropList:
        if prop in specstr:
            continue
        else:
            unusedProp = unusedProp + [prop]

    # if there are unused propositions, print out a warning

    if unusedProp and not fastslow:
       print '##############################################'
       print 'Warning:'
       print 'The following propositions seem to be unused:'
       print unusedProp
       print 'They should be removed from the proposition lists\n'

    return spec,linemap,failed,LTL2LineNo,internal_props


def parseInit(sentence,PropList,lineInd):
    ''' This function creates the LTL formula representing the initial conditions.
        It takes the sentence and PropList - a list of propositions (to check that only 'legal'
        propositions are used) and 'lineInd' that indicates which line is being processed.
        Returns the LTL formula as a string.
    '''

    FalseRE = re.compile('false',re.IGNORECASE)
    TrueRE = re.compile('true',re.IGNORECASE)
    

    if FalseRE.search(sentence):
        # All propositions should be false
        tempFormula = ''
        for prop in PropList:
            tempFormula = tempFormula + '\t\t\t !' + prop + ' & \n'

        LTLsubformula = tempFormula
        return LTLsubformula

    if TrueRE.search(sentence):
        # All propositions should be true
        tempFormula = ''
        for prop in PropList:
            tempFormula = tempFormula + '\t\t\t ' + prop + ' & \n'

        LTLsubformula = tempFormula
        return LTLsubformula
            
    # A more specific initial condition 
    tempFormula = sentence[:]
    
    # Replace logic operations with TLV convention
    tempFormula = replaceLogicOp(tempFormula)

    # checking that all propositions are 'legal' (in the list of propositions)
    for prop in re.findall('([\w\.]+)',tempFormula):
        if not prop in PropList:
            print 'ERROR(1): Could not parse the sentence in line '+ str(lineInd)+' because ' + prop + ' is not recognized\n'
            return ''
    
    
    LTLsubformula = '\t\t\t (' + tempFormula + ') & \n'

    return LTLsubformula

def parseSafety(sentence,sensorList,regionList,actuatorList,customsList,lineInd,CompletionFlag=False):
    ''' This function creates the LTL formula representing a basic safety requirement.
        It takes the sentence, the sensor list and the list of all robot propositions (to check that only 'legal'
        propositions are used and to determine whether it is an environment safety or a robot one)
        and 'lineInd' that indicates which line is being processed.
        Returns a dictionary with 2 keys: 'formula' containing the LTL formula as a string and 'type' containing
        either 'EnvTrans' or 'SysTrans'.
    '''

    formulaInfo = {}
    # Initilalizing in case the sentence cannot be parsed
    formulaInfo['formula'] = ''
    formulaInfo['type'] = ''
    
    tempFormula = sentence[:]
    robotPropList = actuatorList + customsList
    allRobotProp = regionList + robotPropList
    PropList = sensorList + allRobotProp
    
    # Replace logic operations with TLV convention
    tempFormula = replaceLogicOp(tempFormula)
    # checking that all propositions are 'legal' (in the list of propositions)
    if CompletionFlag:
        MatchStr = '((?:finish(?:ed)?\s+)?\(?[\w\.]+\)?)'
        MatchPropStr ='((finish(ed)?\s+)?\(?(?P<prop>[\w\.]+)\)?)'
    else:
        MatchStr = '([\w\.]+)'

    for prop in re.findall(MatchStr,tempFormula):
        originalProp = prop
        if CompletionFlag:
            prop = re.search(MatchPropStr,prop).group('prop')

        if not prop in PropList:
            print 'ERROR(2): Could not parse the sentence in line '+ str(lineInd)+' because ' + prop + ' is not recognized\n'
            formulaInfo['type'] = 'EnvTrans' # arbitrary
            return formulaInfo

        if (prop in sensorList and formulaInfo['type'] == 'SysTrans') or \
           (prop in allRobotProp and formulaInfo['type'] == 'EnvTrans'):
            print 'ERROR(3): Could not parse the sentence in line '+ str(lineInd)+' containing:'
            print sentence
            print 'because both environment and robot propositions are used \n'
            formulaInfo['type'] = 'EnvTrans' # arbitrary
            return formulaInfo

        originalPropPattern = originalProp.replace('(','\(').replace(')','\)')
        if prop in sensorList and formulaInfo['type'] == '':
            formulaInfo['type'] = 'EnvTrans'
            # replace every occurrence of the proposition with next(proposition)
            # it is written this way to prevent nesting of 'next' (as with the .replace method)
            tempFormula = re.sub('(next\('+prop+'\)|\\b'+prop+'\\b)', 'next('+ prop +')',tempFormula)

        elif prop in allRobotProp and formulaInfo['type'] == '':
            formulaInfo['type'] = 'SysTrans'
            # replace every occurrence of the proposition with next(proposition)
            # it is written this way to prevent nesting of 'next' (as with the .replace method)
            if CompletionFlag:
                if prop in regionList:
                    tempFormula = re.sub('(next\('+originalPropPattern+'\)|\\b'+originalPropPattern+'\\b|'+originalPropPattern+')', 'next(e.'+ prop.replace("s.","") +'_rc)',tempFormula)
                elif prop in actuatorList:
                    tempFormula = re.sub('(next\('+originalPropPattern+'\)|\\b'+originalPropPattern+'\\b|'+originalPropPattern+')', 'next(e.'+ prop.replace("s.","") +'_ac)',tempFormula)
                else:
                    tempFormula = re.sub('(next\('+originalPropPattern+'\)|\\b'+originalPropPattern+'\\b|'+originalPropPattern+')', 'next('+ prop +')',tempFormula)
            else:
                tempFormula = re.sub('(next\('+originalPropPattern+'\)|\\b'+originalPropPattern+'\\b|'+originalPropPattern+')', 'next('+ prop +')',tempFormula)
        else:
            # replace every occurrence of the proposition with next(proposition)
            # it is written this way to prevent nesting of 'next' (as with the .replace method)
            tempFormula = re.sub('(next\('+originalPropPattern+'\)|\\b'+originalPropPattern+'\\b|'+originalPropPattern+')', 'next('+ prop +')',tempFormula)
    formulaInfo['formula'] = '\t\t\t [](' + tempFormula + ') & \n'

    return formulaInfo

def parseLiveness(sentence,sensorList,regionList,actuatorList,customsList,lineInd,fastslow=False):
    ''' This function creates the LTL formula representing a basic liveness requirement.
        It takes the sentence, the sensor list and the list of all robot propositions (to check that only 'legal'
        propositions are used and to determine whether it is an environment safety or a robot one)
        and 'lineInd' that indicates which line is being processed.
        Returns a dictionary with 2 keys: 'formula' containing the LTL formula as a string and 'type' containing
        either 'EnvGoals' or 'SysGoals'.
    '''

    
    formulaInfo = {}
    # Initilalizing in case the sentence cannot be parsed
    formulaInfo['formula'] = ''
    formulaInfo['type'] = ''
    
    tempFormula = sentence[:]
    robotPropList = actuatorList + customsList
    allRobotProp = regionList + robotPropList
    PropList = sensorList + allRobotProp

    SafetyCompletionRE = re.compile('^\s*(finished|not finished|have not finished|have finished|do not finish|finish|not finish)',re.IGNORECASE)
    SafetyRE = re.compile('^\s*(always|always do |do|always sense|sense|finished|finish|have)',re.IGNORECASE)
    MatchStr = '((?:finish(?:ed)?\s+)?\(?[\w\.]+\)?)'
    MatchPropStr = '((finish(ed)?\s+)?\(?(?P<prop>[\w\.]+)\)?)'

    # Replace logic operations with TLV convention
    tempFormula = replaceLogicOp(tempFormula)


    for prop in re.findall(MatchStr,tempFormula):
        originalProp = prop

        #check if we are using the 'finished' keyword
        if SafetyCompletionRE.search(prop):
            CompletionFlag = True
        else:
            CompletionFlag = False

        prop = re.search(MatchPropStr,prop).group('prop')

        if not prop in PropList:
            print 'ERROR(4): Could not parse the sentence in line '+ str(lineInd)+' because ' + prop + ' is not recognized\n'
            formulaInfo['type'] = 'EnvGoals' # arbitrary
            return formulaInfo

        if (prop in sensorList and formulaInfo['type'] == 'SysGoals') or \
           (prop in allRobotProp and formulaInfo['type'] == 'EnvGoals'):
            #print 'ERROR(5): Could not parse the sentence in line '+ str(lineInd)+' containing:'
            #print sentence
            #print 'because both environment and robot propositions are used \n'
            #formulaInfo['type'] = 'EnvGoals' # arbitrary
            #return formulaInfo
            pass

        if prop in sensorList and formulaInfo['type'] == '' or CompletionFlag:
            formulaInfo['type'] = 'EnvGoals'

        elif prop in allRobotProp and formulaInfo['type'] == '':
            formulaInfo['type'] = 'SysGoals'

        originalPropPattern = originalProp.replace('(','\(').replace(')','\)')
        if fastslow:
            if CompletionFlag:
                if prop in regionList:
                    #tempFormula = re.sub('(?<=[! &|(\t\n])'+prop+'(?=[ &|)\t\n])', 'e.'+prop.replace('s.','')+'_rc', tempFormula)
                    newProp = 'next(e.'+prop.replace('s.','')+'_rc)'
                elif prop in actuatorList:
                    newProp = 'next(e.'+prop.replace('s.','')+'_ac)'
                else:
                    #tempFormula = re.sub('(?<=[! &|(\t\n])'+prop+'(?=[ &|)\t\n])', 'e.'+prop.replace('s.','')+'_ac', tempFormula)
                    newProp = 'next('+prop+')'
            else:
                if prop in regionList:
                    #tempFormula = re.sub('(?<=[! &|(\t\n])'+prop+'(?=[ &|)\t\n])', 'e.'+prop.replace('s.','')+'_rc', tempFormula)
                    newProp = 'next('+prop+') & next(e.'+prop.replace('s.','')+'_rc)'
                elif prop in actuatorList:
                    newProp = 'next('+prop+') & next(e.'+prop.replace('s.','')+'_ac)'
                else:
                    #tempFormula = re.sub('(?<=[! &|(\t\n])'+prop+'(?=[ &|)\t\n])', 'e.'+prop.replace('s.','')+'_ac', tempFormula)
                    newProp = 'next('+prop+')'

            tempFormula = re.sub('(?<=[! &|(\t\n])'+originalPropPattern+'(?=[ &|)\t\n])|(?<=[! &|(\t\n])'+originalPropPattern +\
                                  '|'+originalPropPattern + '(?<=[! &|(\t\n])', newProp, tempFormula)

    formulaInfo['formula'] = '\t\t\t []<>(' + tempFormula + ') & \n'

    return formulaInfo


def parseConditional(Condition,ReqFormulaInfo,CondType,sensorList,regionList,actuatorList,customsList,lineInd,fastslow=False):
    ''' This function creates the LTL formula representing a conditional.
        It takes the condition, the requirement formula (that was already parsed),
        the condition type, and the list of all propositions (to check that only 'legal'
        propositions are used) and 'lineInd' that indicates which line is being processed.
        Returns a dictionary with 2 keys: 'formula' containing the LTL formula as a string and 'type' containing
        the type of the requirement.
    '''
    robotPropList = actuatorList + customsList
    allRobotProp = regionList + robotPropList
    PropList = sensorList + allRobotProp
    formulaInfo = {}
    # Initilalizing in case the sentence cannot be parsed
    formulaInfo['formula'] = ''
    formulaInfo['type'] = ReqFormulaInfo['type']

    # Getting the subformula encoding the condition
    condFormula = parseCond(Condition,sensorList,regionList,actuatorList,customsList,formulaInfo['type'],lineInd,fastslow)
    if condFormula == '':
        # If could not parse the condition, return
        print 'ERROR(6): Could not parse the condition in line '+ str(lineInd)+'\n'
        return formulaInfo
    
    # Getting the subformula encoding the requirement:
    # First strip and save as formulaStart the '[]' or '[]<>',
    # then remove the '& \n'.
    reqFormulaInd = ReqFormulaInfo['formula'].find('(')
    reqFormula = ReqFormulaInfo['formula'][reqFormulaInd:]
    reqFormula = reqFormula.replace('& \n','')
    formulaStart = ReqFormulaInfo['formula'][:reqFormulaInd]

    # Adding the 2 pieces together
    if CondType == 'IfThen':
        tempFormula = condFormula + ' -> ' + reqFormula
    elif CondType == 'Unless':
        tempFormula = reqFormula  + ' | ' + condFormula
    elif CondType == 'IFF':
        tempFormula = condFormula + ' <-> ' + reqFormula
    else:
        print 'ERROR(7): Could not parse the condition in line '+ str(lineInd)+' because of unknown condition type\n'
        return formulaInfo
    
    
    formulaInfo['formula'] = formulaStart +'(' + tempFormula + ') & \n'

    return formulaInfo


def parseCond(condition,sensorList,regionList,actuatorList,customsList,ReqType,lineInd,fastslow=False):
    ''' This function creates the LTL formula representing the condition part of a conditional.
        It takes the condition and PropList - a list of propositions (to check that only 'legal'
        propositions are used)and 'lineInd' that indicates which line is being processed.
        Returns the LTL formula as a string.
    '''

    NotFlag = False
    NextFlag = False
    EdgeType = None
    CompletionFlag = False
    robotPropList = actuatorList + customsList
    allRobotProp = regionList + robotPropList
    PropList = sensorList + allRobotProp

    tempFormula = ''

    # Flag to indicate whether it is a liveness requirement
    livenessFlag = ReqType == 'EnvGoals' or ReqType == 'SysGoals'
    
    # List of possible condition parts
    regionPastCond = 'the robot was in'+'|'+ 'the robot was not in'+'|' + \
                     'you were in' + '|' + 'you were not in' + '|' + \
                     'it was in' + '|' + 'it was not in' + '|' + \
                     'was in' + '|' + 'was not in' + '|' + \
                     'were in' + '|' + 'were not in' + '|'

    regionCurrCond = 'the robot is in'+'|'+ 'the robot is not in'+'|' + \
                     'you are in' + '|' + 'you are not in' + '|' + \
                     'it is in' + '|' + 'it is not in' + '|' + \
                     'is in' + '|' + 'is not in' + '|' + \
                     'are in' + '|' + 'are not in' + '|'

    sensorPastCond = 'the robot sensed'+'|'+ 'the robot did not sense'+'|' + \
                     'you sensed' + '|' + 'you did not sense' + '|' + \
                     'it sensed' + '|' + 'it did not sense' + '|' + \
                     'you were sensing' + '|' + 'you were not sensing' + '|' + \
                     'sensed' + '|' + 'did not sense' + '|'
  
    sensorCurrCond = 'the robot is sensing'+'|'+ 'the robot is not sensing'+'|' + \
                     'you are sensing' + '|' + 'you are not sensing' + '|' + \
                     'it is sensing' + '|' + 'it is not sensing' + '|' + \
                     'are sensing' + '|' + 'are not sensing' + '|' + \
                     'is sensing' + '|' + 'is not sensing' + '|'

    actionPastCond = 'the robot activated'+'|'+ 'the robot did not activate'+'|' + \
                     'you activated' + '|' + 'you did not activate' + '|' + \
                     'it activated' + '|' + 'it did not activate' + '|' + \
                     'activated' + '|' + 'did not activate' + '|' + \
                     'you were activating' + '|' + 'you were not activating'

    actionCurrCond = 'the robot is activating'+'|'+ 'the robot is not activating'+'|' + \
                     'you are activating' + '|' + 'you are not activating' + '|' + \
                     'it is activating' + '|' + 'it is not activating' + '|' + \
                     'are activating' + '|' + 'are not activating' + '|' + \
                     'is activating' + '|' + 'is not activating' 

    if fastslow:
        actionCompletionPastCond = '|'+'the robot had finished' + '|' + 'the robot had not finished'+'|' +\
                                   'you had finished' + '|' + 'you have not finished' + '|' + \
                                   'it had finished'+ '|' + 'it had not finished' + '|' + \
                                   'had finished' + '|' + 'had not finished'

        actionCompletionCurrCond = '|'+'the robot has finished' + '|' + 'the robot has not finished'+'|' +\
                                   'you have finished' + '|' + 'you have not finished' + '|' + \
                                   'it has finished'+ '|' + 'it has not finished' + '|' + \
                                   'have finished' + '|' + 'have not finished' + '|' + \
                                   'has finished' + '|' + 'has not finished'

    RiseCond = "|".join(["start of", "beginning of"])
    FallCond = "|".join(["end of"])

    PastCond = regionPastCond + sensorPastCond + actionPastCond
    CurrCond = regionCurrCond + sensorCurrCond + actionCurrCond

    if fastslow:
        PastCond = PastCond + actionCompletionPastCond
        CurrCond = CurrCond + actionCompletionCurrCond
    
    possCond = "|".join([PastCond, CurrCond, RiseCond, FallCond]) 

    condRE = re.compile('('+ possCond + ')',re.IGNORECASE)
    subCond = re.split(condRE,condition)

    for subCondition in subCond:

        # If empty string, move on...
        if subCondition == '':
            continue

        # Determine the type of the formula to follow (current)
        # If not a livness requirement, add 'next'
        if (subCondition in CurrCond) and (not livenessFlag) :
            NextFlag = True
            if 'not' in subCondition:
                NotFlag = True
            else:
                NotFlag = False
            EdgeType = None
            if fastslow:
                if (subCondition in actionCompletionCurrCond):
                    CompletionFlag = True
                else:
                    CompletionFlag = False
        # Determine the type of the formula to follow (past)
        # If past OR the requirement is liveness, then do NOT add 'next'
        elif (subCondition in PastCond) or (subCondition in CurrCond):
            NextFlag = False
            if 'not' in subCondition:
                NotFlag = True
            else:
                NotFlag = False
            EdgeType = None
            if fastslow:
                if (subCondition in actionCompletionPastCond + actionCompletionCurrCond):
                    CompletionFlag = True
                else:
                    CompletionFlag = False

        # Determine the type of the formula to follow (edge)
        elif (subCondition in RiseCond):
            NextFlag = False
            NotFlag = False
            EdgeType = "rising"

            if fastslow:
                if (subCondition in actionCompletionPastCond + actionCompletionCurrCond):
                    CompletionFlag = True
                else:
                    CompletionFlag = False

        elif (subCondition in FallCond):
            NextFlag = False
            NotFlag = False
            EdgeType = "falling"

            if fastslow:
                if (subCondition in actionCompletionPastCond + actionCompletionCurrCond):
                    CompletionFlag = True
                else:
                    CompletionFlag = False


        # This is the body of the subcondition
        else:
            # checking that all propositions are 'legal' (in the list of propositions)
            # and adding 'next' if needed
            if fastslow:
                props = re.findall('((?:finished\s+)?\(?[\w\.]+\)?)',subCondition)
            else:
                props = re.findall('([\w\.]+)',subCondition)
            props = [x for x in props if not x in ['and','or']]

            # Add a '!' in front if needed
            if NotFlag:
                subTempFormula = '!(' + subCondition
            else:
                subTempFormula = '(' + subCondition
                     
            # Remove the logic operator (or,and) that connects this subcondition to the next
            # and add it after adding ')', if it exists
            subCondConnect = re.split(' (or|and)\s*$',subTempFormula)
            if len(subCondConnect)==3:
                subTempFormula = subCondConnect[0] + ' ) ' + subCondConnect[1] + ' '
            else:
                subTempFormula = subCondConnect[0] + ' ) '

            # Replace logic operations with TLV convention 
            subTempFormula = replaceLogicOp(subTempFormula)

            for prop in props:
                if not prop in PropList and (not fastslow or not re.search('((finished\s+)?\(?(?P<prop>[\w\.]+)\)?)',prop).group('prop') in PropList):
                    print 'ERROR(8): Could not parse the sentence in line '+ str(lineInd)+' because ' + prop + ' is not recognized\n'
                    return ''

            if EdgeType is not None:
                # make sure edge condition only operates on a stand-alone proposition
                if len(props) != 1:
                    print 'ERROR(8): Could not parse the sentence in line '+ str(lineInd)+' because edge condition (' + subCondition + ') contains more than one proposition\n'
                    return ''

                prop = props[0]

                # only allow edges in places where they make sense
                if livenessFlag:
                    print 'ERROR(8): Could not parse the sentence in line '+ str(lineInd)+' because edge conditions cannot be used in liveness specifications\n'
                    return ''

                if fastslow and 'finished' in prop:
                    propStriped = re.search('((finished\s+)?\(?(?P<prop>[\w\.]+)\)?)',prop).group('prop')
                    if propStriped in regionList:
                        newProp = 'e.' + propStriped.replace('s.','')+'_rc'
                    elif propStriped in actuatorList:
                        newProp = 'e.' + propStriped.replace('s.','')+'_ac'
                    else:
                        print 'ERROR(8): Could not parse the sentence in line '+ str(lineInd)+' because ' + prop + ' is not a region or actuator proposition and cannot be converted into a completion prop\n'
                        return ''
                    subTempFormula = re.sub(prop,newProp,subTempFormula)
                    prop = newProp

                if (prop in allRobotProp) and (ReqType == 'EnvTrans'):
                    print 'ERROR(8): Could not parse the sentence in line '+ str(lineInd)+' because ' + prop + ' is a robot proposition and cannot be used in an edge condition in an environment safety requirement\n'
                    return ''

                if EdgeType == "rising":
                    subTempFormula = re.sub(prop, "!(%s) & next(%s)" % (prop, prop), subTempFormula)
                elif EdgeType == "falling":
                    subTempFormula = re.sub(prop, "(%s) & !next(%s)" % (prop, prop), subTempFormula)

            else:
                for prop in props:
                    if fastslow and 'finished' in prop:
                        propStriped = re.search('((finished\s+)?\(?(?P<prop>[\w\.]+)\)?)',prop).group('prop')
                        if propStriped in regionList:
                            newProp = 'e.' + propStriped.replace('s.','')+'_rc'
                        elif propStriped in actuatorList:
                            newProp = 'e.' + propStriped.replace('s.','')+'_ac'
                        else:
                            print 'ERROR(8): Could not parse the sentence in line '+ str(lineInd)+' because ' + prop + ' is not a region or actuator proposition and cannot be converted into a completion prop\n'
                            return ''
                        subTempFormula = re.sub(prop,newProp,subTempFormula)
                        prop = newProp

                    if NextFlag and (prop in allRobotProp) and (ReqType == 'EnvTrans') and (not fastslow or not CompletionFlag):
                        print 'Warning: In the sentence in line '+ str(lineInd)+' :'
                        print prop + ' is a robot proposition and should be in the past form in an environment safety requirement'
                        print 'The next operator was not added\n'
                        continue

                    if NextFlag:
                        # replace every occurrence of the proposition with next(proposition)
                        # it is written this way to prevent nesting of 'next' (as with the .replace method)
                        prop = prop.replace('(','').replace(')','')
                        subTempFormula = re.sub('(next\('+prop+'\)|'+prop+')', 'next('+ prop +')',subTempFormula)

            if fastslow and CompletionFlag:
                for prop in props:
                    prop = prop.replace('(','').replace(')','')
                    if prop in regionList:
                        subTempFormula = re.sub('(?<=[! &|(\t\n])'+prop+'(?=[ &|)\t\n])', 'e.'+prop.replace('s.','')+'_rc', subTempFormula)
                    elif prop in actuatorList:
                        subTempFormula = re.sub('(?<=[! &|(\t\n])'+prop+'(?=[ &|)\t\n])', 'e.'+prop.replace('s.','')+'_ac', subTempFormula)

            tempFormula = tempFormula + subTempFormula

    LTLsubformula = '(' + tempFormula + ')'

    return LTLsubformula

def parseAfterEachTime(Cond, Requirement, sensorProp, regionList, robotPropList, lineInd, StayFormula, fastslow=False):
    # Getting the subformula encoding the condition
    Cond = parseCond(Cond,sensorProp,regionList,actuatorList,customsList,"SysGoals",lineInd,fastslow)
    if Cond == '':
        # If could not parse the condition, return
        print 'ERROR(6): Could not parse the condition in line '+ str(lineInd)+'\n'
        return '', '', ''

    # parse the liveness requirement
    ReqFormulaInfo = parseLiveness(Requirement,sensorProp,regionList,actuatorList,customsList,lineInd,fastslow)
    if ReqFormulaInfo['formula'] == '': failed = True

    # If not SysGoals, then it is an error
    if not ReqFormulaInfo['type']=='SysGoals':
        print 'ERROR(6): Only system "After each time" statements are currently supported: line '+ str(lineInd)+'\n'
        return '', '', ''

    Req = ReqFormulaInfo['formula'].replace('\t\t\t []<>','')
    Req = Req.rstrip().rstrip("&")


    mem_prop = "m_" + re.sub("(e|s)\.", "", re.sub("\s+","_",(Cond+" without "+Req).replace("&","").replace("(","").replace(")",""))).rstrip("_")

    # Everything seems to be OK, lets write the formulas
    MemoryFormula = '\t\t\t ([]( next(s.' + mem_prop + ') <-> ( (' + nextify(Cond) + ' | s.' + mem_prop + ' ) & !(' + nextify(Req) + '))) ) & \n'    
    GoalFormula = '\t\t\t ([]<>((s.' + mem_prop + ') -> (' + Req + ')) ) & \n'    
    
    # For instantaneous reactivity
    ReactivityFormula = '\t\t\t ([]( (!s.' + mem_prop + ' & next(s.' + mem_prop + ')) -> (' + StayFormula + ' ) ) ) & \n'    
    
    return MemoryFormula + ReactivityFormula, GoalFormula, mem_prop
    


def parseToggle(EventProp,ToggleEvent,sensorProp,regionList,actuatorList,customsList,lineInd,fastslow=False):

    ''' This function creates the LTL formulas encoding when a proposition's value should toggle (T->F, F->T).
        It takes the proposition, the boolean formula defining the toggle event, the propositions
        (to check that only 'legal' propositions are used) and 'lineInd' that indicates which line is being processed.
        Returns the LTL formula as a string.
    '''
    robotPropList = actuatorList + customsList
    RobotProp = regionList + robotPropList
    PropList = sensorProp + RobotProp
    
    # Getting the subformula encoding the condition
    ToggleEvent = parseCond(ToggleEvent,sensorProp,regionList,actuatorList,customsList,"SysTrans",lineInd,fastslow)
    if ToggleEvent == '':
        # If could not parse the condition, return
        print 'ERROR(6): Could not parse the condition in line '+ str(lineInd)+'\n'
        return ''

    # Checking the event proposition
    if EventProp in sensorProp:
        print 'ERROR(11): Could not parse the sentence in line '+ str(lineInd)+' because ' + EventProp + ' is a sensor proposition instead of a robot proposition'
        return ''
    elif EventProp in RobotProp:
        pass
    else:
        # Not a single valid robot proposition
        print 'ERROR(12): Could not parse the sentence in line '+ str(lineInd)+' because ' + EventProp + ' is not a valid robot proposition'
        return ''

    # Everything seems to be OK, lets write the formulas
    FlipOffFormula = '\t\t\t ([](( (' + EventProp + ') & (' + ToggleEvent + ')) -> !next(' + EventProp + ')) ) & \n'    
    FlipOnFormula = '\t\t\t ([](( !(' + EventProp + ') & (' + ToggleEvent + ')) -> next(' + EventProp + ')) ) & \n'    
    HoldOnFormula = '\t\t\t ([](( (' + EventProp + ') & !(' + ToggleEvent + ')) -> next(' + EventProp + ')) ) & \n'    
    HoldOffFormula = '\t\t\t ([](( !(' + EventProp + ') & !(' + ToggleEvent + ')) -> !next(' + EventProp + ')) ) & \n'    
    
    LTLsubformula = FlipOffFormula + FlipOnFormula + HoldOnFormula + HoldOffFormula

    return LTLsubformula

def parseEvent(EventProp,SetEvent,ResetEvent,sensorProp,regionList,actuatorList,customsList,lineInd,fastslow=False):
    ''' This function creates the LTL formulas encoding when a proposition should be true and when false.
        This is used as a macro to define 'memory' propositions.
        It takes the proposition, the boolean formulas defining the set and reset events, the propositions
        (to check that only 'legal' propositions are used) and 'lineInd' that indicates which line is being processed.
        Returns the LTL formula as a string.
    '''

    RobotProp = regionList + actuatorList + customsList
    PropList = sensorProp + RobotProp
    
    # Replace logic operations with TLV convention
    SetEvent = replaceLogicOp(SetEvent)
    ResetEvent = replaceLogicOp(ResetEvent)

    # String to match
    if fastslow:
        MatchPropStr = '((?:finished\s+)?\(?[\w\.]+\)?)'
        StripPropStr = '((finished\s+)?\(?(?P<prop>[\w\.]+)\)?)'
    else:
        MatchPropStr = '([\w\.]+)'
        StripPropStr = '(?P<prop>[\w\.]+)'

    if fastslow:
        regionGroupSetEvent = re.findall('((?:finished\s+\(?)(?P<propGroup>[\w\.\s+?\|?]+)\)?)', SetEvent)
        regionGroupResetEvent = re.findall('((?:finished\s+\(?)(?P<propGroup>[\w\.\s+?\|?]+)\)?)', ResetEvent)

        if regionGroupSetEvent:
            for groupProp in regionGroupSetEvent:
                replacePropGroup = ["finished " + x for x in groupProp[1].replace(" ","").split("|")]
                if len(replacePropGroup) > 1:
                    SetEvent = SetEvent.replace(groupProp[0]," ( " + " | ".join(replacePropGroup)+ " ) ")

        if regionGroupResetEvent:
            for groupProp in regionGroupResetEvent:
                replacePropGroup = ["finished " + x for x in groupProp[1].replace(" ","").split("|")]
                if len(replacePropGroup) > 1:
                    ResetEvent = ResetEvent.replace(groupProp[0], " ( "+" | ".join(replacePropGroup)+" ) ")


    # checking that all propositions are 'legal' in the set and reset events, and adding the 'next' operator
    for prop in re.findall(MatchPropStr,SetEvent):
        propStriped = re.search(StripPropStr,prop).group('prop')
        if not propStriped in PropList:
            print 'ERROR(9): Could not parse the sentence in line '+ str(lineInd)+' because ' + prop + ' in the set event is not recognized\n'
            return ''
        else:
            if propStriped not in RobotProp:
                # replace every occurrence of the proposition with next(proposition)
                # it is written this way to prevent nesting of 'next' (as with the .replace method)
                SetEvent = re.sub('(next\('+prop+'\)|'+prop+')', 'next('+ prop +')',SetEvent)

            if fastslow:
                if "finished" in prop:
                    if propStriped in regionList:
                        SetEvent = re.sub('(?<=[! &|(\t\n])?'+prop.replace('(','\(').replace(')','\)')+'(?=[ &|)\t\n])?', '('+propStriped.replace('s.','e.')+'_rc)', SetEvent)
                    elif propStriped in actuatorList:
                        SetEvent = re.sub('(?<=[! &|(\t\n])?'+prop.replace('(','\(').replace(')','\)')+'(?=[ &|)\t\n])?', propStriped.replace('s.','e.')+'_ac', SetEvent)

    if ResetEvent.upper()=='FALSE':
        ResetEvent = 'FALSE'
    else:
        for prop in re.findall(MatchPropStr,ResetEvent):
            propStriped = re.search(StripPropStr,prop).group('prop')
            if not propStriped in PropList:
                print 'ERROR(10): Could not parse the sentence in line '+ str(lineInd)+' because ' + prop + ' in the reset event is not recognized\n'
                return ''
            else:
                if propStriped not in RobotProp:
                    # replace every occurrence of the proposition with next(proposition)
                    # it is written this way to prevent nesting of 'next' (as with the .replace method)
                    ResetEvent = re.sub('(next\('+prop+'\)|'+prop+')', 'next('+ prop +')',ResetEvent)
                if fastslow:
                    if "finished" in prop:
                        if propStriped in regionList:
                            ResetEvent = re.sub('(?<=[! &|(\t\n])?'+prop.replace('(','\(').replace(')','\)')+'(?=[ &|)\t\n])?', '('+propStriped.replace('s.','e.')+'_rc)', ResetEvent)
                        elif propStriped in actuatorList:
                            ResetEvent = re.sub('(?<=[! &|(\t\n])?'+prop.replace('(','\(').replace(')','\)')+'(?=[ &|)\t\n])?', propStriped.replace('s.','e.')+'_ac', ResetEvent)

    # Checking the event proposition
    if EventProp in sensorProp:
        print 'ERROR(11): Could not parse the sentence in line '+ str(lineInd)+' because ' + EventProp + ' is a sensor proposition instead of a robot proposition'
        return ''
    elif EventProp in RobotProp:
        pass
    else:
        # Not a single valid robot proposition
        print 'ERROR(12): Could not parse the sentence in line '+ str(lineInd)+' because ' + EventProp + ' is not a valid robot proposition'
        return ''

    # Everything seems to be OK, lets write the formulas
    SetFormula = '\t\t\t ([](( (' + SetEvent + ') & !(' + ResetEvent + ')) -> next(' + EventProp + ')) ) & \n'    
    ResetFormula = '\t\t\t ([](  (' + ResetEvent + ') -> !next(' + EventProp + ')) ) & \n'    
    TrueFormula = '\t\t\t ([](( (' + EventProp + ') & !(' + ResetEvent + ')) -> next(' + EventProp + ')) ) & \n'    
    FalseFormula = '\t\t\t ([](( !(' + EventProp + ') & !(' + SetEvent + ')) -> !next(' + EventProp + ')) ) & \n'    
    
    LTLsubformula = SetFormula + ResetFormula + TrueFormula + FalseFormula

    return LTLsubformula


def replaceLogicOp(formula):
    ''' This function replaces the logic operators with TLV convention.
    '''

    # Replace logic operations with TLV convention
    andRE = re.compile('\\band\\b',re.IGNORECASE)
    orRE = re.compile('\\bor\\b',re.IGNORECASE)
    impliesRE = re.compile('\\bimplies\\b',re.IGNORECASE)
    iffRE = re.compile('\\biff\\b',re.IGNORECASE)
    notRE = re.compile('\\bnot\\b',re.IGNORECASE)
    
    formula = andRE.sub(' & ',formula)
    formula = orRE.sub(' | ',formula)
    formula = impliesRE.sub(' -> ',formula)
    formula = iffRE.sub(' <-> ',formula)
    formula = notRE.sub(' ! ',formula)

    return formula

def replaceRegionName(formula,bitEncode,regionList):
    ''' This function replaces the region names with the appropriate bit encoding.
    '''
    tempFormula = formula[:]
    
    # first replace all 'next' region names with the next encoding
    for nextProp in re.findall('(next\(s\.\w+\)|next\(\(s\.\w+\)\))',tempFormula):
        prop = nextProp.replace('next((s.','').replace('next(s.','').replace(')','')

        if prop in regionList:
            ind = regionList.index(prop)
            tempFormula = tempFormula.replace(nextProp, bitEncode['next'][ind])
            # 'replace' is fine here because we are replacing next(region) and that cannot be a partial name
               
    # replace all leftover region names with the current encoding
    for prop in re.findall('s\.(\w+)',tempFormula):
        if prop in regionList:
            ind = regionList.index(prop)
            # replace every occurrence of the proposition with the bit encoding
            # it is written this way to prevent partial word replacements (as with the .replace method)
            #tempFormula = re.sub('\s+'+prop, ' '+bitEncode['current'][ind],tempFormula) # if following a space
            #tempFormula = re.sub('\('+prop, '('+bitEncode['current'][ind],tempFormula) # if in ()
            tempFormula = re.sub('\\bs\.'+prop+'\\b', bitEncode['current'][ind],tempFormula)

            #tempFormula = tempFormula.replace(prop, bitEncode['current'][ind])

    # Handle region sensor names. first replace all 'next' region names with the next encoding
    for nextProp in re.findall('(next\(e\.\w+\)|next\(\(e\.\w+\)\))',tempFormula):
        prop = nextProp.replace('next((e.','').replace('next(e.','').replace(')','')

        if prop in regionList:
            ind = regionList.index(prop)
            tempFormula = tempFormula.replace(nextProp, bitEncode['envNext'][ind])
            # 'replace' is fine here because we are replacing next(region) and that cannot be a partial name

    # Handle region sensor names
    for prop in re.findall('e\.(\w+)',tempFormula):
        if prop in regionList:
            ind = regionList.index(prop)
            # replace every occurrence of the proposition with the bit encoding
            # it is written this way to prevent partial word replacements (as with the .replace method)
            tempFormula = re.sub('\\be\.'+prop+'\\b', bitEncode['env'][ind],tempFormula)
    
    LTLsubformula = tempFormula 

    return LTLsubformula

def createStayFormula(regionNames, use_bits=True, fastslow=False):
    if use_bits:
        numBits = int(math.ceil(math.log(len(regionNames),2)))
        if fastslow:
            tempFormula = '( (next(e.sbit0) <-> next(s.bit0)) '
        else:
            tempFormula = '( (next(s.bit0) <-> s.bit0) '

        for bitNum in range(1,numBits):

            # Encoding the string
            if fastslow:
                tempFormula = tempFormula + '& (next(e.sbit'+ str(bitNum) +') <-> next(s.bit'+ str(bitNum) +') ) '
            else:
                tempFormula = tempFormula + '& (next(s.bit'+ str(bitNum) +') <-> s.bit'+ str(bitNum) +') '
        
        StayFormula = tempFormula + ')'

        return StayFormula
    else:
        if fastslow:
            return "({})".format(" & ".join(["(next(e.{0}_rc) <-> next({1}))".format(rn.replace("s.",""),rn) for rn in regionNames]))
        else:
            return "({})".format(" & ".join(["({0} <-> next({0}))".format(rn) for rn in regionNames]))


def bitEncoding(numRegions,numBits):
    ''' This function creates a dictionary that contains the bit encoding for the current
        and next region. Takes number of regions and returns a dictionary with 'current' \
        and 'next' as keys, each containing a list of the respective encodings.
    '''

    # initializing the dictionary
    bitEncode = {}

    # create an encoding of the regions, current and next for sys and current for env
    currBitEnc = []
    nextBitEnc = []
    envBitEnc = []
    envNextBitEnc = []
    for num in range(numRegions):
        binary = numpy.binary_repr(num) # regions encoding start with 0
        # Adding zeros
        bitString = '0'*(numBits-len(binary)) + binary

        # Encoding the string
        envTempString = '('
        currTempString = '('
        nextTempString = '('
        envNextTempString = '('
        for bitNum in range(numBits):
            if bitNum>0:
                envTempString = envTempString + ' & '
                currTempString = currTempString + ' & '
                nextTempString = nextTempString + ' & '
                envNextTempString = envNextTempString + '&'
            if bitString[bitNum]=='1':
                envTempString = envTempString + 'e.sbit' + str(bitNum)
                currTempString = currTempString + 's.bit' + str(bitNum)
                nextTempString = nextTempString + 'next(s.bit' + str(bitNum) + ')'
                envNextTempString = envNextTempString + 'next(e.sbit' + str(bitNum) + ')'
            if bitString[bitNum]=='0':
                envTempString = envTempString + '!e.sbit' + str(bitNum)
                currTempString = currTempString + '!s.bit' + str(bitNum)
                nextTempString = nextTempString + '!next(s.bit' + str(bitNum) + ')'
                envNextTempString = envNextTempString + '!next(e.sbit' + str(bitNum) + ')'

        envTempString = envTempString + ')'
        currTempString = currTempString + ')'
        nextTempString = nextTempString + ')'
        envNextTempString = envNextTempString + ')'
        
        envBitEnc.append(envTempString)
        currBitEnc.append(currTempString)
        nextBitEnc.append(nextTempString)
        envNextBitEnc.append(envNextTempString)

    bitEncode['env'] = envBitEnc
    bitEncode['current'] = currBitEnc
    bitEncode['next'] = nextBitEnc
    bitEncode['envNext'] = envNextBitEnc

    return bitEncode

