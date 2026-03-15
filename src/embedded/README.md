# Source code for GNC controller C++17

## Compile and run

### Preparation
Optional:
```
rm -rf build
```

### Build
Create the build files
```
cmake -G "MinGW Makefiles" -B build
```

Compile the code
```
cmake --build build
```

### Run
Run unit tests
```
./build/run_gnc_tests.exe
```
