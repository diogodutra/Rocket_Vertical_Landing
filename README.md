# Answers
Answer to the assignment questions are available in `docs` folder.

# Justifications
- Test-driven code development approach.
- Time-propagation of state is non-linear for accuracy. Any further assumptions are reserved for controllers design only.
- Time integrator should not be Euler to avoid numerical errors (ie: artificial non-conservation of energy).

# Notes:
48 hours does not seem enough for the scope of this project with all usual development processes, so some simplifications were put in place:
- No academic formalities (ie: literature references).
- Dynamically typed language (ie: no type definitions) as optiization is not the focus.
- First theoretical gains were not quite meeting the landing requirements (ie: vertical speed higher). Isolated altitude.
- Found out that vel_error was not correctly accounted for in my first Altitude controller. Fixed it.
- Classical reversed signals in Attitude controller. Fixed it.
- My position controller was unstable at first due to unstable nested loop. Artificially decreased theta_cmd from outer loop output.
- Landing opportunity sometimes missed (too early or too late). So, I added a terminal landing modification in z_ref. Ideally it should have been a state machine with virtual gates, but not much time left.