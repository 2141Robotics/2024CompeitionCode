# 2141 2024 Comp Code

### Getting Started

Only supported on Windows10+. Please setup the WPILib fork of vscode [here](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html).
```
.\get_started.ps1
```

### Spotless Linting

Our CI includes a job called `spotless` which does some formatting checks to make sure all our codebase looks the same. If you get an error here and want to quickly fix it,
run `.\gradlew spotlessApply` and commit the changes.


### Quail Submodule
 
To make developing with Quail easier, we have a submodule of it located in `repositories/quail` with a symbolic link in our robot code to the needed quail files. 

The way submodules are tracked in git is by a specific commit, so if you make changes to quail make sure you commit them in quail, and then commit the update to the submodule in this repo. For example:

```
cd repositories/quail
git commit -a -m "Quail changes..."
cd ../../
git commit -a -m "Updating quail submodule"
```

Whenever you change branches here, make sure your submodule is up to date with:

```
git submodule update
```