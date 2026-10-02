# Tests

There are two kinds of tests, with different purposes. Both use JUnit: each method marked `@Test` is one test, and `assertEquals(expected, actual)` fails the test if the two differ.

**Unit tests** check logic by itself. They hand a mechanism a fake IO (a few lines of code that pretends to be the hardware), set up a situation, and check what the mechanism decides. For example, `ArmTest` tells the arm it's resting on the hard stop, enables the robot, and checks that the arm doesn't try to lift itself. No motors, physics, or waiting, so they finish in a fraction of a second. This is the IO layer paying off again: the same split that makes replay possible makes the logic easy to test.

**Simulation tests** (tagged `sim`) run the real robot code against the simulator, in real time, because the simulated Phoenix devices run on a real clock. They check that everything works together: the drive goes where it's told, odometry agrees with where the robot really is, the arm reaches its targets and holds against gravity, and vision pulls a wrong pose estimate back to the truth. The suite takes about a minute.

Run them from VS Code's terminal (on Windows, `gradlew.bat` instead of `./gradlew`), or with the "Run Test" link above any test method:

```bash
./gradlew unitTest   # just the fast unit tests, a few seconds
./gradlew test       # everything, about a minute
```

`./gradlew build` runs neither, to keep builds quick. Run the tests before pushing; GitHub runs all of them on every push either way.

[Back to the README](../README.md)
