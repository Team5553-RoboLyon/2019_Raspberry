# Vision processing for First Deep Space 2019
Team 5553 Robo'Lyon vision code running on a raspberry for First Deep Space.

# Getting started

Follow instructions of https://github.com/Team5553-RoboLyon/LyonVision-Template


# Usage

## Running your code locally on your computer :
```bash
.\gradlew runVision
```

## Deploying your code to the Raspberry Pi
```bash
.\gradlew deploy
```

## Build all
```bash
.\gradlew build
```

## Test code
```bash
.\gradlew check
```


# Git Submodule

The folder `src/lib/` is a git submodule. It is a link to the [Team5553-RoboLyon/LyonVision-Library](https://github.com/Team5553-RoboLyon/LyonVision-Library) repo where the pseudo-library files are located. These files are in a separated repository because there can be used by several projects.
