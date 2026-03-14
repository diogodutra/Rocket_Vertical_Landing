# Answers
Answer to the assignment questions are available in `docs` folder.

# Environment
All develoment was perfored in VS Code on Windows 11.

# Run the Python code
Single flight simulation:
```
python main.py
```

Grid search on initial state disturbances:
```
python -m src.helper.run_grid
```

Unit tests:
```
pytest tests/test_altitude_controller.py
pytest tests/test_attitude_controller.py
pytest tests/test_all_controllers.py
```

# Run the C++ code
Refer to `src/embedded/README.md`.

# Notes:
48 hours does not seem enough for the entire scope of this project with all usual development processes, so some simplifications were put in place:
- No academic formalities (ie: literature references).
- Python remains dynamically typed language (ie: no type definitions) as optimization is not the focus.
- Test-driven code development approach was followed.
- First theoretical gains were not quite meeting the landing requirements (ie: vertical speed higher), so further manual tuning was performed.
- Found out that `vel_error` was not correctly accounted for in my first Altitude controller implementation. Fixed it.
- Typical reversed signals in Attitude controller. Fixed it.
- My Position controller was unstable at first due to coupling from nested loop. Fixed it by artificially decreased $\theta_{cmd}$ from outer loop output.
- Landing opportunity sometimes missed (too early or too late). So, I added a terminal landing modification in z_ref. This could be improved even further, though. Ideally, it should have been a state machine with virtual gates for different flight phases, but not much time left.