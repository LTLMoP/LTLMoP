# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pick_up, 1
drop, 1
radio, 0
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
carrying_item

RegionFile: # Relative path of region description file
../firefighting/floorplan.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
fire, 1
person, 0
hazardous_item, 1


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
#SIMPLE EXAMPLES OF EACH CASE OF UNSYNTHESIZABILITY:

##############################################
##UNSATISFIABLE
##############################################

##System safety unsatisfiable -- single step
Env starts with false
Robot starts in deck
Always porch
Always drop

##Environment safety unsatisfiable -- single step
#Env starts with false
#Robot starts in deck
#Always person and not person


##Multi-step unsatisfiability
#Env starts with false
#Robot starts with not drop
#If you activated drop then do not drop
#If you activated drop then do drop
#If you did not activate drop then do drop

##############################################

##System liveness and safety unsatisfiable
#Env starts with false
#Robot starts in deck
#Always not porch
#Infinitely often porch

##Environment liveness and safety unsatisfiable
#Env starts with false
#Robot starts in deck
#Always not person
#Infinitely often person

##############################################
##UNREALIZABLE
##############################################

##System safety unrealizable -- single step
#Env starts with false
#Robot starts in deck
#Always not porch
#If you are sensing person then do porch

##Environment safety unrealizable -- single step
#Env starts with false
#Robot starts in deck
#Always not person
#If you were in porch then do  person

##Multi-step unrealizability
#Do pick_up if and only if you are sensing hazardous_item
#Do drop if and only if you were in porch
#If you are activating pick_up or you activated pick_up then stay there
#If you are activating drop or you activated drop then stay there
#If you did not activate pick_up then always not porch


##############################################

##System liveness unrealizable
#Env starts with false
#Robot starts in deck
#Visit porch
#If you are sensing fire then do not porch

##Environment liveness unrealizable
#Env starts with false
#Robot starts in deck
#Visit porch
#If you are sensing person then do not porch
#Visit not person
#If you were in deck then do person

