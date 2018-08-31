# A Controller for the Crazyflie #

For this optional "project" you will be applying the concepts you've learned to creating a controller for the crazyflie drone through the udacidrone API.


## Control Architecture ##

The crazyflie exposes two levels of control: velocity commands and attitude commands.  As there are two possible options, the development of an offboard controller will be done in two major steps for ease of testing and demonstration:

 - [outer loop controller](#outer-loop-controller)
     + [altitude controller](#altitude-controller)
     + [lateral position controller](#lateral-position-controller)

 - [inner loop controller](#inner-loop-controller)
     + [attitude](#attitude)
     + [thrust](#thrust)

The first controller that will be built is the outer loop controller which will compute velocity commands that can be sent to the crazyflie and consists of an altitude controller and a lateral position controller.  See the figure below for a schematic of this controller.

![outer loop controller schematic](crazyflie_outer.png?raw=true "Outer Loop Schematic")

The second controller that will be built is the inner loop controller which computes attitude/thrust commands from the velocity commands computed by the outer loop controller.  This then feeds those attitude/thrust commands to the crazyflie.  See the figure below for a schematic of this controller.

![inner loop controller schematic](crazyflie_outer_inner.png?raw=true "Inner Loop Schematic")

For the most part these controllers are very similar to the ones from the controls project, with a few exceptions.  The altitude controller and lateral position controller will just stop at velocity commands, we won't be calculating all the way to accelerations.  The yaw controller will be exactly the same as what was seen in the controls project as we are once again controlling yaw rate with the crazyflie.  The velocity controller is new and this is where we will control velocity by commanding attitudes.

As we build the controller step by step, we will actually be leveraging the fact that the crazyflie already has a controller written for it.  This will allow us to only introduce one part of the controller at the time and let the existing controller handling the other elements for us.  For example, as we build the altitude controller, we will be leaving lateral velocity control to the crazyflie so it will maintain 0 velocity in the lateral directions enabling us to focus entirely on the altitude controller.

## Repository Structure ##

This repository contains the skeleton code (`master` branch) for developing a controller for the crazyflie and a solution version (`solution` branch) that contains a tuned implementation of the controllers.

**NOTE: as this control is done offboard the crazyflie, there may be some variation in performance from one computer to another, so you may find that the tuning on the solution may not work best for your computer.**

There are two sets of files in the repository:

 - `*_flyer.py` - these files contain implementations of the Udacidrone `Drone` class that handle managing the link and controlling the crazyflie.  When it comes to running the code, these are the files you run.  Make sure you read through all the comments within the code for a full description of the functionality and configurations that are possible.

 - `*_controller.py` - these files contain the controllers themselves (and is where you will find TODOs in the `master` branch to be completed).  There is one controller file for each the outer loop and inner loop controllers.


## Outer Loop Controller ##

The outer loop controller will be built in 2 steps, first we will build the altitude controller and then we will build the lateral position controller.  Each of these can be run and tested separately to make the building and tuning processes easier.

### Altitude Controller ###

In this first step, we will be building an altitude controller to the level of commanding velocities.  Later on we will continue to develop the altitude controller to compute thrust itself, but for now we will stop at vertical velocity as the crazyflie allows us to control this directly.

Much like in the project, we will create a PID controller on the altitude to command velocity, so the resulting command should look like the first step of your altitude controller from the controls project:

```python
def altitude_controller(self, alt_cmd, alt, hdot_cmd=0.0):
    hdot_cmd += self._kp_alt * (alt_cmd - alt)  # compute desired vertical velocity from altitude error
    hdot_cmd = np.clip(hdot_cmd, -self._hdot_max, self._hdot_max)  # saturate as desired
    return hdot_cmd
```

*NOTE: for the crazyflie a simple P controller is all that will be necessary, however try adding the I and D terms and see how it changes the controller!*

And that's it!  Now to [choose a gain](#break-an-aside-on-gain-selection) and see what happens!

Once you have decided on a gain, you can run this outer loop controller using the `velocity_flyer.py` script as follows:

```sh
python velocity_flyer.py --uri radio://0/80/2M
```

Where the uri passed in should be the uri you configured for your crazyflie.

*NOTE: if you leave the lateral position gain (`self._kp_pos`) set to 0, then only the altitude controller will run, allowing you to focus tuning the altitude controller alone.  The crazyflie may drift slowly as it is only trying to maintain 0 lateral velocity, not hold a position, but the drift should be fairly slow.*

**IMPORTANT NOTE: if at any time you need to stop the script and control of the crazyflie, if ctrl+c does not work, simply unplug the crazyradio dongle from your computer and the crazyflie will go into "stop" mode after a couple seconds (this will make it fall out of the sky).**

### Lateral Position Controller ###

Now that we have our altitude controller, let's handle the lateral position.  Once again we will be using a PID controller on the position, so it will be the first (TODO: check how many lines) line of the controller you made for the controls project:

```python
def lateral_position_control(self, pos_cmd, pos, vel_cmd):

    lateral_vel_cmd = self._kp_pos * (pos_cmd[0:2] - pos[0:2]) + vel_cmd[0:2]  # compute lateral velocity command from position error
    lateral_vel_cmd = np.clip(lateral_vel_cmd, -self._v_max, self._v_max)  # saturate as desired
    return lateral_vel_cmd

```

*NOTE: for the crazyflie a simple P controller is all that will be necessary, however try adding the I and D terms and see how it changes the controller!*

And that's it!  Now we just need to choose a starting gain and see how it works.

Once you have decided on a gain, you can run this outer loop controller using the `velocity_flyer.py` script as follows:

```sh
python velocity_flyer.py --uri radio://0/80/2M
```

Where the uri passed in should be the uri you configured for your crazyflie.

### Break: An Aside on Gain Selection ###

Now that you've coded up your controller, it's time to pick some initial gains and see how it works.  For the course, we knew the exact properties of our drone in the simulator, allowing us to calculate some initial gains mathematically, but here, that's not so much the case.  So let's see if we can build some intuition and do some back of the envelope calculation to decide on some gains.

First things first, let's make sure we don't ever try and go too fast and add a limit on the allowable velocity command.  Let's keep things nice and slow and limit the velocity to 0.3 m/s.

As a recap, our controller is calculating the position error (how far we are from where we'd like to be) and multiplying that by our gain (`self._kp_pos` and `self._kp_alt`) to generate a velocity command.  In general, when we are far away from our target position (`pos_error >> 0`), we'd like to approach it at our max velocity, so let's not worry about that case since we've added a mechanism to limit our velocity command to a max velocity.  What really starts to come in to play here is at what distance do you want to start slowing down.  The easiest way to start making these decisions is looking at examples, so let's make up a few.

For example, let's set our max velocity to 1 m/s and set `_kp_pos = 0.5`, let's take a look at what it means for our crazyflie's velocity profile as it approaches the waypoint:

 - at 1 meter away, we will be flying 0.5 m/s
 - at 0.5 meters away, we will be flying at 0.25 m/s

Now let's say we set `_kp_pos = 10`, let's take a look at what it means:

 - at 1 meter away, we will be flying at 10 m/s (or max velocity)
 - at 0.5 meters away, we will be flying at 5 m/s (or max velocity)

*For those who need a bit more of a visualization, go ahead and plot it out, you will see that as you increase your gain, the slope of the resulting velocity can get impossibly steep!*

Do you think that the crazyflie will be able to stop without overshooting if it's still flying at 5 m/s when it's 0.5 meters away from the target position?
    
 > If the crazyflie could respond immediately to commands and instantaneously change its velocity vector, then yes, this would be possible, but alas, in the real world, we have delays, it takes some time to change velocity (and attitude), which results in needing to tune our controller.  Different drones will be able to react at different rates, allowing for much more aggressive control on some drones versus others.

Do you think you could approach the target position faster than 0.25 m/s when 0.5 meters out?

These are the questions that you should be asking yourself as you choose an initial set of gains to work with for your controller.  You should err on the side of caution and go with the option that gives you room to increase your gain to get better performance rather than have to decrease your gain.

As a last part to build some of our intuition, we can see that setting the max speed value allows us to have very aggressive control without the proportional, potentially impossibly high, speed increase when `pos_error >> 0`.

In the next controller (the inner loop controller), this same thought process can also be used to get an idea for what kind of gain might seem like a reasonable starting point.  It gets a little trickier as the units get a bit less intuitive (attitude will all be in radians), but the same intuition can give you a helping hand!

**It is worth mentioning that in this case there really isn't such a thing as too small of a value, worse case your crazyflie will just not go anywhere, but it also won't fall out of the sky.  When it comes to lower level control loops, for example an attitude controller that is responsible for keeping your drone level, you will start seeing cases where you can have a gain that is too low and results in your drone falling out of the sky!**

## Inner Loop Controller ##

Now that we have made a working outer loop controller that is successfully able to fly the crazyflie using velocity commands, let's build an inner loop controller that will take those velocity commands and generate attitude/thrust commands to control the crazyflie.

You may be thinking "why would I want to command attitude, when commanding a velocity gives me a good enough position controller!"  And you would be right in thinking that!  If your goal is solely to command a position in space and you have a drone that can do accurate velocity commands, then you may find you don't need to go this far.  But, what if you want to be able to not just go from point A to point B, but do so with specific orientations in space?  For example, what if you wanted to go through a window and you knew you needed to be perfectly level going through the window, or more interestingly, at a specific angle?  This starts to go into the realm of flying specific trajectories, which we will get to a little later, but provides a little insight as to why you might end up commanding at this level if you do have velocity commands at your disposal.  (Remember there is always the case where your drone only supports attitude commands!)

While the entire inner loop will be tested at once, we will build it in two steps:

 1. compute attitude commands from the horizontal velocity command
 2. compute the thrust command (total thrust) from the vertical velocity command

### Attitude ###

We have already generated a velocity command for our position error, so let's build on that and use the cascaded structure we saw in the controls projects to generate an attitude command from our velocity command:

```python
def velocity_controller(self, vel_cmd, vel):

    pitch = -self._kp_vel * (vel_cmd[0] - vel[0])  # note the sign change!  Remember + pitch is up, meaning it will send out drone backwards!
    roll = self._kp_vel * (vel_cmd[1] - vel[0])

    # add some limits
    pitch_cmd = np.clip(pitch, -self._bank_max, self._bank_max)
    roll_cmd = np.clip(roll, -self._bank_max, self._bank_max)

    ...
```

### Thrust ###

The second half of the velocity controller is to convert the vertical velocity to a thrust command.  Instead of true thrust, we will need to send a normalized thrust command, that is, a value between 0 and 1 to represent the thrust level, with 0 being no thrust and 1 being the max amount of thrust possible.  This does mean we need to know how much thrust, in [N], is the max possible thrust of the crazyflie.  We've done that calculation for you and you can find it as a constant at the top of the `inner_controller.py` file.

So let's again use the same process as the controls project to compute the thrust:

```python
def velocity_controller(self, vel_cmd, vel)
    ...

    accel_cmd = self._kp_hdot * (hdot_cmd - hdot)  # compute acceleration from vertical velocity error
    accel_cmd = np.clip(accel_cmd, -self._haccel_max, self._haccel_max)  # saturate as desired
    thrust_cmd_N = DRONE_M * (accel_cmd + GRAVITY_MAG) / (np.cos(pitch_cmd) * np.cos(roll_cmd))  # compute thrust in N positive up

    # need to normalize the thrust
    thrust_cmd = thrust_cmd_N / MAX_THRUST_N

    return pitch_cmd, roll_cmd, thrust_cmd
```

### Flying it! ###

Once you have decided on a gain, you can run this inner loop controller using the `attitude_flyer.py` script as follows:

```sh
python attitude_flyer.py --uri radio://0/80/2M
```

Where the uri passed in should be the uri you configured for your crazyflie.

## Flying Trajectories ##

Now that we have a controller and it is flying simply waypoint missions, the next area of interest is flying trajectories, which is an area we have helped set up but will be leaving for you to explore!

We have provided a framework for reading in a trajectory file (see below for the file format) and a sample very simple straight line and back trajectory.  In addition you will find a `trajectory_flyer.py` file that contains the necessary class and script to test it out.

Go ahead and give it a try!  You should see the same performance you saw with the straight line waypoint sets that are found in the `velocity_flyer.py` and `attitude_flyer.py` files.  This is not unsurprising as if you look at the trajectory file itself, it is just about the same information.

### Trajectory File Format ###

A trajectory file, as currently defined by the handler class, is a text file with each line being a trajectory point.  Each line contains the following 4 pieces of information as comma separated values:

 - relative time (in flight time in [s])
 - north position (in [m])
 - east position (in [m])
 - down position (in [m])


### Further Challenges ###

While the trajectory file we have provided you is a very simple one, it should be enough to get you started in the right direction to further explore the world of trajectories and explore the limits that you can take your crazyflie!

To get you started, here are some small modifications ideas to get a feel for how trajectories can be of use:

 - **Dynamic flights speeds.**  Using the same idea of a simple line, can you modify your trajectory file to do the return trip twice as quickly?  How about starting and ending slowly but going quickly in the middle?

 - **Altitude variation.**  Can you have your crazyflie fly a pattern such as a sine wave in height while flying the line?  Or how about always climbing quickly but descending slowly?

For those of you who are really ambitious, here are some ideas of how you might be able to extend the code provided:

 - **Create your own complex trajectory files.**  How about flying a figure 8?  How quickly can you get your crazyflie to fly a given trajectory?  How tight of turns, or how complex of trajectories can your controller handle?

 - **Add attitude information to trajectory.**  Extend the trajectory handler and the file format to include attitude to each trajectory point.  Can you successfully achieve specific attitudes at specific times during the flight?

*Note: As you make more complicated trajectories, you may find that you will need to retune your controller to get the best performance you can out of the crazyflie.*

## Single Controller Option ##

In the previous two parts, we built our controller as an outer loop computing velocity commands from position information and an inner loop computing attitude/thrust commands from velocity information.  This was the same approach that was used in the controls project.

Here we will just introduce the idea of a different structure to the controller.  We have not provided example code for this structure, this is food for thought for those of you who want to explore other control structures and see how they might behave differently on a real platform.

Instead of building it as two parts, it is possible to build the entire controller as one piece:

```python
attitude_cmd = KpPos * (pos_cmd - pos) + KpVel * (vel_cmd - vel)
```

If we are not flying a trajectory, we set `vel_cmd = 0`, which means that we are constantly damping our control based on how fast we are currently traveling.  This structure ends up being a little more like a PD controller on position, but we are still directly measuring velocity instead of differentiating position.  Note that with this structure, we can't directly limit the velocity command that is created, we can only limit the attitude command.

For this one, we won't be walking through what the solution looks like directly, but rather leaving it up to you to play around with the controller and see what happens!

For those who are very ambitious, can you think of different control structures you could use?  The crazyflie can be quite forgiving, so give it a try and see how it works out!


