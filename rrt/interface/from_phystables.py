"""Interface for loading phystables trials"""
from ..space import WallSpace
from ..goals import BoxGoal2D
import warnings
import numpy as np

def load_phystable_trial(trial, goaltypes=[], ball_expand=True):
    """Loads in a phystables trial to use RRT on

    Currently outputs a WallSpace... may need to update for odd walls

    Args:
        trial (SimpleTrial): a trial from phystables for loading
        goaltypes ([returns]): a set of onreturns for the goals to turn into
                               goals for RRT
        ball_expand (bool): Should walls be expanded to account for ball size?

    Returns:
        A WallSpace analog of the trial
    """
    try:
        from phystables import SimpleTrial
    except:
        raise ImportError("phystables package is required for loading")

    assert isinstance(trial, SimpleTrial), "Must load a phystables trial"

    if len(trial.abnormwalls) > 0:
        warnings.warn("Abnormal walls will be ignored (not yet implemented)")

    # Go through the walls & goals to parse into Space walls & goals
    glist = []
    wlist = []
    if ball_expand:
        expd = trial.ball[2]
    else:
        expd = 0

    for w in trial.normwalls:
        wlist.append([np.array(w[0]) - expd, np.array(w[1]) + expd])
    for g in trial.goals:
        if g[2] in goaltypes:
            glist.append(BoxGoal2D(np.array(g[0]) - expd,
                                   np.array(g[1]) + expd))
        else:
            # Unloved goals become walls
            wlist.append([np.array(g[0]) - expd,
                          np.array(g[1]) + expd])

    return WallSpace(trial.dims, glist, wlist)
