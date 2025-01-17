#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from kinsolve import *
import numpy as np

def sweepCoords(baseCoords, sweepParams, axisSweep):
    
    x, y, z = baseCoords.origin
    swept_coords = []

    if axisSweep == "X":
        values = np.arange(x + sweepParams["X"]["start"],
                           x + sweepParams["X"]["stop"] + sweepParams["X"]["step"],
                           sweepParams["X"]["step"])
        for val in values:
            swept_coords.append([val, y, z])  # Only sweep X
    elif axisSweep == "Y":
        values = np.arange(y + sweepParams["Y"]["start"],
                           y + sweepParams["Y"]["stop"] + sweepParams["Y"]["step"],
                           sweepParams["Y"]["step"])
        for val in values:
            swept_coords.append([x, val, z])  # Only sweep Y
    elif axisSweep == "Z":
        values = np.arange(z + sweepParams["Z"]["start"],
                           z + sweepParams["Z"]["stop"] + sweepParams["Z"]["step"],
                           sweepParams["Z"]["step"])
        for val in values:
            swept_coords.append([x, y, val])  # Only sweep Z

    return swept_coords


def main():

    """ Suspension Points """
    # In form of Point([x,y,z])
    # Wheel_Center
    # Y point of wc should be track width / 2
    # Wheel_Center
    wc = Point([0,	622.5,	203])
    # Lower Wishbone
    lfi = Point([175.1,	    175,	111]) # Lower_Fore_Inner
    lai = Point([-175.1,	175,	111]) # Lower_Aft_Inner
    lo  = Point([-3.1,	    608,	114]) # Lower_Upright_Point
    # Upper Wishbone
    ufi = Point([120.1,	    240,	223]) # Upper_Fore_Inner
    uai = Point([-120.1,	240,	216]) # Upper_Aft_Inner
    uo  = Point([-7.13,	    595,	299]) # Upper_Upright_Point
    # Tie Rod or Steering Rod
    tri = Point([55.1, 140, 163]) # Tie_Rod_Inner
    tro = Point([55.1, 600, 163]) # Tie_Rod_Outer
    
    unit = "mm"  # used in graph axis labels, not used in code (yet...)
    
    # Pushrod or Pullrod Points
    # The P-rod inner point is the outboard (usually) point of the rocker/bellcrank
    pri = Point([ -20.3792, 350  ,  487.934 ])
    pro = Point([ -13.9192, 556.8442,  124.9426])
    
    # Rocker Center of Rotation
    rkr = Point([ -23.38  , 280     ,  450])
    
    # Shock Pickup Points (upper, lower)
    # The shock upper point is the inner (usually) point of the rocker/bellcrank
    sku = Point([ -25     , 150     ,  490])
    skl = Point([ -30     , 150     ,  300])

    """ Suspension Setup """
    # Full jounce and rebound mark the bounds for the solver
    # if they are too large, and cannot be achieved with your linkage system
    # the code will not throw an error but will either not finish solving or give erroneous results
    full_jounce = 25.4
    full_rebound = -25.4
    
    # toe, camber and caster are used for static offsets on the graphs
    # these will not affect the solver
    toe = 0
    camber = 0
    caster = 0
    
    #User selects which var to sweep
    print("Which variable would you like to sweep?")
    print("Options: X, Y, Z")
    axisSweep = input("Enter your choice: ").strip().upper()

    # Validate input
    if axisSweep not in {"X", "Y", "Z"}:
        print("Invalid choice. Please choose X, Y, or Z.")
        return
    
    sweepConfig = {
        "X": {"start": -1, "stop": 1, "step": 0.5},
        "Y": {"start": -1, "stop": 1, "step": 0.5},
        "Z": {"start": -1, "stop": 1, "step": 0.5}
    }

    # Generates swept coordinates
    swept_lfi_coords = sweepCoords(lfi, sweepConfig, axisSweep)
    
    for coords in swept_lfi_coords:
        print(f"Processing LFI coords: {coords}")
        lfi = Point(coords)
    
    kin = KinSolve(
        wheel_center=wc,
        lower_wishbone=(lfi, lai, lo),
        upper_wishbone=(ufi, uai, uo),
        tie_rod=(tri, tro),
        
        p_rod=(pri, pro),
        rocker=rkr,
        shock=(skl, sku),

        full_jounce=full_jounce,
        full_rebound=full_rebound,

        unit=unit,
    )
    
    """ Solver Parameters """
    # number of steps in each direction, so a value of 10 will yield 20 datapoints
    # algorithm runs fast enough that its fine to use 1000+, but 100 is just as accurate
    # and it will result in a comprehensible amount of data
    num_steps = 100
    
    kin.solve(
        steps=num_steps,
        offset_toe=toe,
        offset_camber=camber,
        offset_caster=caster
    )
    
    """Link Force Solver Inputs"""
    # NOT VALIDATED YET
    Fx = 0 # N
    Fy = 0 # N
    Fz = 100 # N
    kin.linkforce(Fx, Fy, Fz)
    
    
    """ Plot """
    kin.plot(
        suspension=True,  # Visualize the corner
        
        bump_steer=False,  # Bump Steer vs vertical travel
        bump_steer_in_deg=False,  # Sets y-axis of bump steer plot to roll angle in deg

        camber_gain=False,  # Camber Gain vs vertical travel
        camber_gain_in_deg=False,  # Sets y-axis of camber gain plot to roll angle in deg

        caster_gain=False,  # Caster gain plot
        caster_gain_in_deg=False,  # Sets y-axis of caster gain plot to roll angle in deg

        scrub_gain = False, # Scrub change plot
        scrub_gain_in_deg = False,    # Sets y-axis of scrub gain plot to roll angle in deg

        roll_center_in_roll=False,  # Path of roll center as the car rolls
        
        motion_ratio=False, # Motion Ratio vs vertical travel
        motion_ratio_in_deg=False # Sets y-axis of motion ratio plot to roll angle in deg
    )


if __name__ == "__main__":
    main()