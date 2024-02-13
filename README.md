# 2141 2024 Comp Code

### Getting Started

Only supported on Windows10+. Please setup the WPILib fork of vscode [here](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html).
```
.\get_started.ps1
```

### Spotless Linting

Our CI includes a job called `spotless` which does some formatting checks to make sure all our codebase looks the same. If you get an error here and want to quickly fix it,
run `.\gradlew spotlessApply` and commit the changes.
