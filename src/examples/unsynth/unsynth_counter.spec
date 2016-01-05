# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pick_up, 0
drop, 0
radio, 1
extinguish, 0

CompileOptions:
convexify: True
parser: structured
symbolic: False
use_region_bit_encoding: True
synthesizer: jtlv
fastslow: False
decompose: True

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
../firefighting/floorplan.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
fire, 1
person, 1
hazardous_item, 0


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
living = p4
deck = p7
porch = p3
dining = p6
bedroom = p8
others = 
kitchen = p5

Spec: # Specification in structured English
#Simple specification demonstrating liveness unrealizability
#Environment can win by alternating fire and person

Env starts with false
Robot starts with false
Robot starts in deck

Visit porch

if you are sensing fire then do not living
if you are sensing person then do not kitchen

#always not living
#always not kitchen

always not radio
if you were activating radio then do radio

always not (fire and person)

#Always kitchen or porch or deck or bedroom or dining or living

