# Writing a Controller for the Crazyflie #

Throughout the course, we've handled a variety of different levels of control of a drone, from commanding positions in the first project (Backyard Flyer) to commanding motor thrusts themselves in the third project (Controls).  When it comes to real drones, you will come across interfaces that allow a varying level of control, meaning, you will come across drones that only open position control to the user, some that allow velocity control, and even some that allow acceleration control from "offboard" the drone, meaning not as part of the core control system.  As an aside, the level of control used in the third project (motor thrusts) is typically only ever seen within the core controller of the drone itself, so odds are if you are writing code for that, you're working on the main control system itself!

When it comes to the crazyflie, it opens up a couple levels of control for us to work with: velocity and attitude.  In this optional lesson, we will go through and see how to apply the concepts you've learned in the controls section to build a lateral position, altitude, velocity and yaw controller for your crazyflie, first using the velocity command API and then using the attitude command API.

Unfortunately, because the level of commands available to us are different than the simulator from the controls project, for the most part, we can't directly use that script to control the crazyflie, however we will be using the same concepts and a similar control structure for the crazyflie.

TODO: add a figure of the control structure for the crazyflie controller

Much like the controls project, we will be building this one controller at a time.  We will construct it in the following order:

 - outer loop controller
     + altitude controller
     + lateral position controller

 - inner loop controller
     + velocity controller

For the most part these controllers are very similar to the ones from the controls project, with a few exceptions.  The altitude controller and lateral position controller will just stop at velocity commands, we won't be calculating all the way to accelerations.  The yaw controller will be exactly the same as what was seen in the controls project as we are once again controlling yaw rate with the crazyflie.  The velocity controller is new and this is where we will control velocity by commanding attitudes.

As we build the controller step by step, we will actually be leveraging the fact that the crazyflie already has a controller written for it.  This will allow us to only introduce one part of the controller at the time and let the existing controller handling the other elements for us.  For example, as we build the altitude controller, we will be leaving lateral velocity control to the crazyflie so it will maintain 0 velocity in the lateral directions unabling us to focus entirely on the altitude controller.

## Repository Structure ##

TODO: add some details on how the repo is structured

TODO: add some details on what files the code should be written in, etc

target format:
 - a controller file that is empty that can be used by students as the starting point
 - a controller for each of the specific cases below
 - a script file for each of the specific cases below???
 - a script file for the students as a starting point????


## Altitude Controller ##

In this first step, we will be building an altitude controller to the level of commanding velocities.  Later on we will continue to develop the altitude controller to compute thrust itself, but for now we will stop at vertical velocity as the crazyflie allows us to control this directly.

Much like in the project, we will create a PID controller on the altitude to command velocity, so the resulting command should look like the first step of your altitude controller from the controls project:

```python
def altitude_controller(self, alt_desired, alt):
    # TODO: make this a copy of the real code
    hdot_cmd = self.kp_alt * (alt_desired - alt)
    return hdot_cmd
```

*NOTE: for the crazyflie a simple P controller is all that will be necessary, however try adding the I and D terms and see how it changes the controller!*

Recall from the controls project it can be nice to saturate your commands when you can to make sure that you don't exceed some thresholds, either performance thresholds or safety ones.

```python
# TODO: code to saturate the command
```

And that's it!  Now to choose a gain and see what happens!

**For an example script of just the altitude controller working to maintain altitude, check out the `script name here` script in the `repo name here` repository on github.**



## Lateral Position Controller ##

Now that we have our altitude controller, let's handle the lateral position.  Once again we will be using a PID controller on the position, so it will be the first (TODO: check how many lines) line of the controller you made for the controls project:

```python
def lateral_position_controller(self, pos_desired, pos):
    # TODO make this a copy of the real code
    vel_cmd = self.kp_pos * (pos_desired - pos)
    ...
    return vel_cmd

```

*NOTE: for the crazyflie a simple P controller is all that will be necessary, however try adding the I and D terms and see how it changes the controller!*

Once again we can saturate the velocity commands as desired:

```python
# TODO: add code for saturating the command, for both altitude and lateral position
```

And that's it!  Now we just need to choose a starting gain and see how it works...

### Running Lateral Position Controller ###

There are two ways that the lateral position controller can be run:

 1. letting the crazyflie control the altitude - see TODO: add script name -> NOT POSSIBLEE

 2. running both altitude and lateral position controllers - see TODO: add script name


## Break: An Aside on Gain Selection ##

Now that you've coded up your controller, it's time to pick some initial gains and see how it works.  For the course, we knew the exact properties of our drone in the simulator, allowing us to calculate some initial gains mathematically, but here, that's not so much the case.  So let's see if we can build some intuition and do some back of the envelope calculation to decide on some gains.

First things first, let's make sure we don't ever try and go too fast and add a limit on the allowable velocity command.  Let's keep things nice and slow and limit the velocity to 1 m/s (TODO: decide on good starting limit here).

As a recap, our controller is calculating the position error (how far we are from where we'd like to be) and multiplying that by our gain `KpPos` to generate a velocity command.  In general, when we are far away from our target position (`pos_error >> 0`), we'd like to approach it at our max velocity, so let's not worry about that case since we've added a mechanism to limit our velocity command to a max velocity.  What really starts to come in to play here is at what distance do you want to start slowing down.

For example, let's say we have a max velocity of 1 m/s and we set `KpPos = 0.5`.  That means that anything further than 2 meters from our target, we will be flying at max speed, and once we are within 2 meters we will be slowing down until we finally reach the waypoint.


For example, let's say we set `KpPos = 0.5`, let's take a look at what it means for our crazyflie's velocity profile as it approaches the waypoint:

 - at 1 meter away, we will be flying 0.5 m/s
 - at 0.5 meters away, we will be flying at 0.25 m/s

TODO: add plot with several different gain values to illustrate the velocity profile as we approach the waypoint.

Now let's say we set `KpPos = 10`, let's take a look at what it means:

 - at 1 meter away, we will be flying at 10 m/s (or max velocity)
 - at 0.5 meters away, we will be flying at 5 m/s (or max velocity)

Do you think that the crazyflie will be able to stop without overshooting if it's still flying at 5 m/s when it's 0.5 meters away from the target position?
    
    If the crazyflie could respond immediately to commands and instantaneously change its velocity vector, then yes, this would be possible, but alas, in the real world, we have delays, it takes some time to change velocity (and attitude), which results in needing to tune our controller.  Different drones will be able to react at different rates, allowing for much more aggressive control on some drones versus others.

Do you think you could approach the target position faster than 0.25 m/s when 5 meters out?

These are the questions that you should be asking yourself as you choose an initial set of gains to work with for your controller.  You always want to err on the side of caution and go with the option that gives you room to increase your gain to get better performance rather than have to decrease your gain.


So to build some of our intuition, we can see that setting the max speed value allows us to have very aggressive control without the proportional speed increase when `pos_error >> 0`.

## Velocity Controller ##

Now that you've written a controller commanding velocity, let's go one step further and command attitude.  You may be thinking "why would I want to command attitude, when commanding a velocity gives me a good enough position controller!"  And you would be right in thinking that!  If your goal is solely to command a position in space and you have a drone that can do accurate velocity commands, then you may find you don't need to go this far.  But, what if you want to be able to not just go from point A to point B, but do so with specific orientations in space?  For example, what if you wanted to go through a window and you knew you needed to be perfectly level going through the window, or more interestingly, at a specific angle?  This starts to go into the realm of flying specific trajectories, which we will get to a little later, but provides a little insight as to why you might end up commanding at this level if you do have velocity commands at your disposal.  (Remember there is always the case where your drone only supports attitude commands!)

We have already generated a velocity command for our position error, so let's build on that and use the cascaded structure we saw in the controls projects to generate an attitude command from our velocity command:

```python
def velocity_controller(self, vel_cmd, vel):

    # TODO: make this actually what controller will be.

    pitch_cmd = -KpVelN * (vel_cmd[0] - vel[0])  # note the sign change - since + pitch is upwards, which would send us backwards, not forwards
    roll_cmd = KpVelE * (vel_cmd[1] - vel[1])

    ...
```

TODO: Once again we will saturate our roll and pitch commands to make sure they don't get too extreme.

### Updating Altitude Controller ###

Now that we've moved on to sending attitude commands, we also need to update our altitude controller to send thrust commands instead of velocity commands.  In the controls project, the altitude controller computed all the way to thrust, but for the crazyflie, we will have this level of control done in our velocity controller.

Instead of true thrust, we will need to send a normalized thrust command, that is, a value between 0 and 1 to represent the thrust level, with 0 being no thrust and 1 being the max amount of thrust possible.  This does mean we need to know how much thrust, in [N], is the max possible thrust of the crazyflie.  We've done that calculation for you and you can find it as a constant at the top of the `TODO: file name` file.

So let's again use the same process as the controls project to compute the thrust:

```python

def velocity_controller(self, vel_cmd, vel)
    ...

    accel_cmd = np.clip(accel_cmd, -self._haccel_max, self._haccel_max)
    thrust_cmd_N = DRONE_M * (accel_cmd + GRAVITY_MAG) / (np.cos(pitch_cmd) * np.cos(roll_cmd))

    thrust_cmd = thrust_cmd_N / MAX_THRUST

    return pitch_cmd, roll_cmd, thrust_cmd

```


### Choosing an Initial Gain ###

We are commanding attitude in radians so this is a little harder intuitively, but let's for a moment pretend we are working with degrees (and then we will adjust for the fact that we did our back of the envelope calculation in degrees).

Once again, let's look at what the attitude command profile looks like as our velocity error changes:


Again we can see that maybe starting with a smaller value (around XXX) may be a safer bet than starting with too large of a value.

**It is worth mentioning that in both of these cases there really wasn't such a thing as too small of a value, worse case your crazyflie was just not going anywhere but it still stayed airborne.  When it comes to lower level control loops, for example an attitude controller that is responsible for keeping your drone level, you will start seeing cases where you can have a gain that is too low and results in your drone falling out of the sky!**



## Attitude Commands Version 2 ##

In the previous section we built our attitude command with a cascaded controller, but I wanted to bring up the idea that a different structure would also work for this, specifically:

```python
attitude_cmd = KpPos * (pos_error) + KpVel * (vel_error)
```

In this case, with `vel_error = 0 - vel` (or `vel_cmd - vel` if flying a trajectory).  This means that we are constantly damping our control based on how fast we are currently traveling.  This structure ends up being a little more like a PD controller on position, but we are still directly measuring velocity instead of differentiating position.  Note that with this structure, we can't directly limit the velocity command that is created, we can only limit the attitude command.

For this one, we won't be walking through what the solution looks like directly, but rather leaving it up to you to play around with the controller and see what happens!

For those who are very ambitious, can you think of different control structures you could use?  The crazyflie can be quite forgiving, so give it a try and see how it works out!


## Flying Trajectories ##

Now that you have two different types of controllers to play around with, start to explore the idea of flying specific trajectories.  Can you fly not just a set of waypoints, but maybe a trajectory that commands specific velocities along the way?  Maybe something like the figure 8 that was flown in simulation?  How about holding specific attitudes along the way?

In the `controller.py` file, you will see we've set up a couple of functions that are designed to read in trajectory files structured as follows:

TODO: decide on a trajectory file structure.

We've provided you with one simple trajectory that you can feel free to try out, but now it is up to you to explore the limits that you can take your crazyflie!