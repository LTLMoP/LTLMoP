# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
camera, 1
gripper, 1

CompileOptions:
convexify: True
parser: structured
symbolic: False
use_region_bit_encoding: True
synthesizer: slugs
fastslow: True
decompose: True

CurrentConfigName:
basicSim

Customs: # List of custom propositions
actionDone
setReset1
setReset2

RegionFile: # Relative path of region description file
structuredFastSlow.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
person, 1
ball, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
others = 
r1 = p3
r2 = p2
r3 = p4, p5

Spec: # Specification in structured English
# -- Vasu's Example in ICRA fastslow -- #
Env starts with person
robot starts in r1 with not camera

if you are sensing person then do camera
if you have finished camera then do camera
if you have finished person then do camera

group regions is r1,r3
visit all regions
# ------------------------------------------------ #

#####################
### other if else clauses ###
#####################
#if you had finished r2 then sense ball
#if you are sensing ball then finished camera

###################
### set reset clauses ###
###################
#setReset1 is set on ball and finished r3 and reset on finished camera and finished r2
#setReset2 is set on ball and gripper and reset on false

####################
### stay there clauses ###
####################
#always not ball
#if you are sensing ball then stay there
#if you are sensing start of camera then stay there
#if you are sensing start of finished camera then stay there

#################
### toggle clauses ###
#################
#actionDone is toggled on finished camera or finished gripper

######################
### at least once clauses ###
######################
#if you are sensing person then visit r2 at least once

# --- clauses that don't work with fastslow --- #
#after each time do camera then visit r2

