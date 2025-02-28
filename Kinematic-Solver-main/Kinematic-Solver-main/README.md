# Kinematic-Solver
* Created By Zheer Seyan, with enhancements contributed by Amina Omonullaeva.
* Limited to pure heave and pure roll (for now)
* pair of scripts each for double wishbone and macpherson strut
  * macpherson stuff is prefixed w/ strut
* See Naming_Convention for double wishbone points naming convention used in this script
* Grad descent version (v0 in archive folder) is deprecated but will still work

## How to use
#### Double Wishbone
* just download kinsolve.py and main.py into the same folder
* open main.py and input your parameters
* run main.py and enjoy:)

#### MacPherson Strut
* Stored in MacphersonStrut folder
* just download strutsolve.py and strutmain.py into the same folder
* open strutmain.py and input your parameters
* run strutmain.py and then send me what errors you get
* Slower and less accurate due to eariler version of the grad descent algo.
	* Probably gonna stay that way because it's good enough

## Assumptions Made by the Code
* Corner sees pure heave or roll
* In roll-based analysis, the vertical travel of the side input into the vehicle is equaled in the opposite direction on the opposite side of the car
* Suspension components are infinitely stiff and there is no compliance

## Future additions
* working on a link force solver, check back... at some point in the future
* Report Generator to help organize all the information


## Known bugs
* Roll center is off by a few mm. This is due to tolerance stackup in the calculations. I don't think its worth chasing this down, as the behavior is correct, and the magintude is only off by a few mm
* fixed with better grad descent algo
* strut math all works out p well except the roll center is all wrong and the algo spits out junk sometimes
* Docs and doc website are out of date, Brian made those at some point and they haven't kept up with code changes and comment additions
