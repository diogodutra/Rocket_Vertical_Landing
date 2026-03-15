# Repo
https://github.com/diogodutra/RocketLab_Assignment

# Answers
Answer to the assignment questions are available at `docs\Answers.md`.

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
- 48 hours does not seem enough for the entire scope of this project with all usual development processes, so some simplifications were put in place and some other areas did not receive enough attention yet (ie: machine state for gated path, more unit tests, C++ simulation, automated gains fine-tuning, optional questions, etc.).
- No academic formalities (ie: literature references).
- Python remains dynamically typed language (ie: no type definitions) as optimization is not the focus.
- Test-driven code development approach was followed.
- First theoretical gains were not quite meeting the landing requirements (ie: vertical speed higher), so further manual tuning was performed after 1st theoretical calculations.
- Typical reversed signals in Attitude controller. Fixed it.
- My Position controller was unstable at first due to coupling from nested loop. Fixed it by increasing difference between its frequency and Attitude controller's one.
- Optimal landing opportunity sometimes missed (real landing happens too early or too late). So, I added a minor terminal landing modification in z_ref. This should be more sophisticated, though. Ideally, it should have been a state machine with virtual gates for different flight phases, but unfortunatelly not much time left.