# ChairBot

This is the code for our multi-season chair bot.

## [`ucpp`](https://github.com/nikitakit/ucpp) Setup

If you want to deploy this code to your robot using `ucpp`, first ensure that you have `ucpp` installed on your system.

Once you have verified that your installation is good, clone the repository and change into the directory that it was cloned into.
The rest of these instructions assume that you are in that directory.

Next, run the following commands:

```sh
# Initializes the ucpp tree
$ ucpp init

# Generates a makefile for the project
$ ucpp configure

# Compiles the project
$ make

# Deploys the code to the robot
$ make deploy

```
