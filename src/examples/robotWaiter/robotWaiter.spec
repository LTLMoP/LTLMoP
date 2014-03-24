# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
deliver, 1
pick_up, 1

CompileOptions:
convexify: False
fastslow: False

CurrentConfigName:
BasicSim

Customs: # List of custom propositions
cam_needs_food
jim_needs_food
carrying_food

RegionFile: # Relative path of region description file
robotWaiter.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
food_ready, 1
cam_order, 1
jim_order, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r4 = p3
r2 = p5
Jim = p9
Cam = p10
others = p1
Table = p8
kitchen = p7

Spec: # Specification in structured English
robot starts with false
infinitely often food_ready

# How to get food
carrying_food is set on kitchen and pick_up and reset on deliver
if you are not sensing food_ready then do not pick_up

# We can only carry one thing at a time
if you are activating carrying_food then do not pick_up

# We need food to deliver
if you are not activating carrying_food then do not deliver

# Satisfy all orders
cam_needs_food is set on cam_order and reset on Cam and deliver
jim_needs_food is set on jim_order and reset on Jim and deliver
infinitely often not cam_needs_food
infinitely often not jim_needs_food

# Instantaneous reaction stuff
if you were activating start of cam_needs_food then stay there
if you were activating start of jim_needs_food then stay there

