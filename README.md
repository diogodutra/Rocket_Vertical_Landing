
# Repository
https://github.com/diogodutra/RocketLab_Assignment

# Answers
Answer to the assignment questions are available at [docs\Answers.md](docs\Answers.md).

# Source Code
## Environment
All develoment was performed in VS Code on Windows 11.

## Documentation
Python code follows Google Style while C++ code follows DOxygen.

## Run Python code
Single flight simulation:
```
python -m src.script.run_flight
```

Grid search on initial state disturbances:
```
python -m src.script.run_grid
```

Create flight animation:
```
python -m src.script.create_animation
```

Unit tests:
```
pytest tests/test_altitude_controller.py
pytest tests/test_attitude_controller.py
pytest tests/test_all_controllers.py
```

## Run C++ code
Refer to `src/embedded/README.md`.

# Notes:
48 hours does not seem enough for the entire scope of this project with all usual development processes, so some simplifications were put in place:
- future studies should consider: machine state for gated path; extensive unit tests; C++ full flight simulation for  extensive Monte Carlo (ie: SILS/HILS); automated fine-tuning of gains; and answering the optional questions.
- More tests are necesary to assess GNC robustness, including finer granularity of grid search and Monte Carlo with coupled disturbances.
- No academic formalities (ie: literature references).
- Python remains dynamically typed language (ie: no type definitions) as optimization is not the focus.
- Test-driven code development approach was followed at minimum, nonetheless.

Some challenges perceived during development:
- There is no requirement about the vertical speed during touchdown, so it was defined as 0.5 m/s.
- First theoretical gains were not quite meeting the landing requirements (ie: vertical speed higher), so further basic manual tuning was performed after 1st theoretical calculations.
- Typical reversed signals in Attitude controller. Fixed it.
- My Position controller was unstable at first due to coupling from nested loop. Fixed it by increasing difference between its frequency and Attitude controller's one.
- Optimal landing opportunity sometimes missed (real landing happens too early or too late). So, a minor modification in z_ref was added for smoother terminal landing. This should have been more sophisticated, though. It should have been at least a nested loop where the outter controller defines the vertical descent speed. Another more sophisticated solution would have been a state machine with virtual gates for different flight phases.