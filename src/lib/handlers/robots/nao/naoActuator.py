#!/usr/bin/env python
"""
===============================================
naoActuator.py - Nao Actuator Handler
===============================================

Control predefined Nao motion besides walking
"""

import time
import threading

class naoActuatorHandler:
    def __init__(self, proj, shared_data):
        """

        """
        self.naoInitHandler = shared_data['NAO_INIT_HANDLER']

        self.ledProxy = None
        self.ttsProxy = None
        self.audioProxy = None
        self.audioFileIDs = {}
        self.behaviorProxy = None

        self.doCount = False
        self.asyncBehaviorFlags = {}
        self.asyncBehaviorThreads = {}


    #####################################
    ### Available actuator functions: ###
    #####################################

    def LEDs(self, FaceLED, EarLED, BrainLED, ChestLED, FeetLED, actuatorVal, initial=False):
        """
        Use Nao's LEDs to indicate state of proposition.
        FaceLED (bool): if the eye leds are used. (default=False)
        EarLED (bool): if the ear leds are used. (default=False)
        BrainLED (bool): if the head leds are used. (default=False)
        ChestLED (bool): if the chest led is used. (default=False)
        FeetLED (bool): if the feet leds are used. (default=False)
        """

        if initial:
            if self.ledProxy is None:
                self.ledProxy = self.naoInitHandler.createProxy('ALLeds')
        else:
            if actuatorVal == True:
                if FaceLED == True:
                    self.ledProxy.setIntensity('FaceLeds',1)
                if EarLED == True:
                    self.ledProxy.setIntensity('EarLeds',1)
                if BrainLED == True:
                    self.ledProxy.setIntensity('BrainLeds',1)
                if ChestLED == True:
                    self.ledProxy.setIntensity('ChestLeds',1)
                if FeetLED == True:
                    self.ledProxy.setIntensity('FeetLeds',1)

            else:
                if FaceLED == True:
                    self.ledProxy.setIntensity('FaceLeds',0)
                if EarLED == True:
                    self.ledProxy.setIntensity('EarLeds',0)
                if BrainLED == True:
                    self.ledProxy.setIntensity('BrainLeds',0)
                if ChestLED == True:
                    self.ledProxy.setIntensity('ChestLeds',0)
                if FeetLED == True:
                    self.ledProxy.setIntensity('FeetLeds',0)

    def _countingThread(self, start, skip, period):
        number = start
        while 1:
            if self.doCount:
                self.ttsProxy.say(str(number))
                number = number + skip
                time.sleep(period)
            else:
                # Reset the count
                number = start
                time.sleep(0.1)

    def countOutLoud(self, start, skip, period, actuatorVal, initial=False):
        if initial:
            if self.ttsProxy is None:
                self.ttsProxy = self.naoInitHandler.createProxy('ALTextToSpeech')
            countThread = threading.Thread(target = self._countingThread, args = (start, skip, period))
            countThread.start()
        else:
            if actuatorVal == True:
                self.doCount = True
            else:
                self.doCount = False

    def sayPhrase(self, phrase, actuatorVal, initial=False):
        """
        Use Nao's test to speech system to speak a word.

        phrase (string): The word to be spoken
        """
        if initial:
            if self.ttsProxy is None:
                self.ttsProxy = self.naoInitHandler.createProxy('ALTextToSpeech')
        else:
            if actuatorVal == True:
                self.ttsProxy.say(phrase)

    def playSoundFile(self, filename, actuatorVal, initial=False):
        if initial:
            if self.audioProxy is None:
                self.audioProxy = self.naoInitHandler.createProxy('ALAudioPlayer')
            self.audioFileIDs[filename] = self.audProxy.loadFile(filename)
        else:
            if actuatorVal == True:
                self.audProxy.play(self.audioFileIDs[filename])
                # TODO: Make this pause dynamic to the length of the clip
                time.sleep(1)

    def _killBehaviors(self):
        bhv = self.behaviorProxy.getRunningBehaviors()
        for b in bhv:
            print "Killing already running behavior: " + str(b)
            self.behaviorProxy.stopBehavior(b)

    def runBehavior(self, startBehaviorName, endBehaviorName, repeat, repeat_period, actuatorVal, initial=False):
        """
        Run a behavior that has been pre-loaded onto Nao with Choregraphe.

        startBehaviorName (string): name of behavior to run when prop goes from False to True
        endBehaviorName (string): name of [optional] behavior to run when prop goes from True to False (default="")
        repeat (bool): choose whether to continuously repeat the startBehavior as long as prop is true [asynchronously] (default=false)
        repeat_period (float): period with which to repeat the action, if `repeat` is True.  Must be longer than the action length.
        """

        if initial:
            if self.behaviorProxy is None:
                self.behaviorProxy = self.naoInitHandler.createProxy('ALBehaviorManager')

            # Preload behaviors to make sure they execute quickly
            self.behaviorProxy.preloadBehavior(startBehaviorName)
            if endBehaviorName != "":
                self.behaviorProxy.preloadBehavior(endBehaviorName)

            if repeat:
                self.asyncBehaviorFlags[startBehaviorName+","+endBehaviorName] = False

                def actionThread(self):
                    while True:
                        if self.asyncBehaviorFlags[startBehaviorName+","+endBehaviorName]:
                            self._killBehaviors()
                            self.behaviorProxy.post.runBehavior(startBehaviorName)
                            time.sleep(repeat_period)
                        else:
                            time.sleep(0.1)

                self.asyncBehaviorThreads[startBehaviorName+","+endBehaviorName] = threading.Thread(target = actionThread, args = (self,))
                self.asyncBehaviorThreads[startBehaviorName+","+endBehaviorName].start()
        else:
            if actuatorVal == True:
                if repeat:
                    self.asyncBehaviorFlags[startBehaviorName+","+endBehaviorName] = True
                else:
                    self._killBehaviors()
                    self.behaviorProxy.runBehavior(startBehaviorName)
            else:
                if repeat:
                    self.asyncBehaviorFlags[startBehaviorName+","+endBehaviorName] = False

                self._killBehaviors()
                if endBehaviorName != "":
                    self.behaviorProxy.runBehavior(endBehaviorName)


