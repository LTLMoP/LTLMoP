# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
camera, 1

CompileOptions:
convexify: True
parser: ltl
symbolic: False
use_region_bit_encoding: True
synthesizer: slugs
fastslow: True
decompose: True

CurrentConfigName:
twoRegions

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
twoRegions.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
person, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r1 = p2
r2 = p1
others = 

Spec: # Specification in structured English
# SPEC
# visit r1 and r2 infinitely often
# camera stays on once it is turned on
# camera is triggered when sensing a person
(e.r1_rc & ! e.camera_ac)

[](next(e.r1_rc) <-> !next(e.r2_rc))

[](next(e.r1_rc) | next(e.r2_rc))

[]((e.r1_rc & s.r1)-> next(e.r1_rc))
[]((e.r2_rc & s.r2)-> next(e.r2_rc))

[]<>(s.r1  -> next(e.r1_rc))
[]<>(s.r2  -> next(e.r2_rc))


[]((e.camera_ac & s.camera) -> next(e.camera_ac))
[]((!e.camera_ac & !s.camera) -> !next(e.camera_ac))

--
s.r2

[] (next(e.person) -> next(s.camera))
[] (e.camera_ac -> next(s.camera))

[]<> (e.r1_rc)
[]<> (e.r2_rc)

