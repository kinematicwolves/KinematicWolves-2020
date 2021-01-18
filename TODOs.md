## Housekeeping items

1. Test code on the fall_cleanup branch
    - This should not change or break anything, just some code maintenance
    - The controller mappings are in RobotContainer.java (in configureButtonBindings method)
    - Let's make sure all of these that should work do work

To test the branch, open up git bash and change to the robot's code repository (Look where it is using the file explorer)

```bash

cd ~/Documents/<path to robot code>/KinematicWolves-2020

```

- Get the latest changes from github (this should include the fall_cleanup branch)

```bash

git pull origin 

```

- Checkout the fall_cleanup branch

```bash

git checkout fall_cleanup

```

- Build and deploy robot code, test all the controller functions

**If everything is performing as expected, merge pull request [#37](https://github.com/kinematicwolves/KinematicWolves-2020/pull/37) on Github**

## New work, adding code to work with the additional sensors

- Look at implementation outlined with issue [#38](https://github.com/kinematicwolves/KinematicWolves-2020/issues/38)
