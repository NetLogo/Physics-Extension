# NetLogo Physics Extension (phys)

## Background:

The NetLogo physics extension provides a simple way to simulate physics on agents and patches within models. Currently, to do physics simulation in a model, a great deal of code is required to handle motion, collisions and gravity. In some cases it is possible to copy this code from model to model to implement physics simulations without having a deep level of understanding of the code itself, but this process can still be fairly complicated. The physics extension allows a modeler to interact with a physics simulation that runs outside of the model with a handful of intuitive NetLogo primitives without having to worry about any complicated physics code.  

---

## How it Works:

The NetLogo physics extension creates a physics simulation using the Dyn4J Java physics library that runs outside of the NetLogo model.  The modeler can then create representations of turtles and patches in this external physics simulation. The representation of agents and patches in the external simulation are linked to agents and patches in the model such that changes to the position of them in either will produce a change in position in the other. The modeler can then tell the physics simulation to simulate physics on the representation of the turtles and patches for a given period of time. After this, the positions of the agents and patches are propagated back into the NetLogo model. The physics extension also does collision detection and can provide information about velocities of turtles in the simulation. 

---

## Getting Started:

First, make sure to include the physics extension in the model as such:

```
extensions[phys]
```

Generally one would create physical objects in a `setup` procedure in NetLogo as follows:

```
to setup
  clear-all
  ask patches with [ ; ask patches that form a bounding box around the view
    pycor = min-pycor
    or pycor = max-pycor
    or pxcor = max-pxcor
    or pxcor = min-pxcor
  ]
  [ ; change their color to grey and make them physical
    set pcolor grey 
    phys:set-physical true
  ] 
  ask n-of 10 patches with [
    pycor > min-pycor + 2 
    and pycor < max-pycor - 2
    and pxcor > min-pxcor + 2
    and pxcor < max-pxcor - 2
  ] 
  [ sprout 1 ; spawn some turtles randomly
    [
      set shape "circle"
      set size random 2 + 2 
      set color blue 
      phys:set-physical true ; make the turtles physical
    ]
  ]
  phys:set-gravity 0 -0.4 ; set the global gravity in the simulation
  reset-ticks
end
``` 

Finally one would make `go` procedure that calls `phys:update` on each tick as such:

```
to go
  phys:update 0.0001 ; update the physics simulation by 0.0001 seconds
  tick
end
```

_This code is from the Ball Bounce Demo model._


---

## Demo Models:

---

- [Ball Bounce Demo](https://raw.githubusercontent.com/Loafie/netlogo_phys_extension/tree/master/demos/ball_bounce_demo.nlogo)

- [Target Practice Demo](https://raw.githubusercontent.com/Loafie/netlogo_phys_extension/master/demos/target_practice_demo.nlogo)

- [Color Collision Demo](https://raw.githubusercontent.com/Loafie/netlogo_phys_extension/tree/master/demos/color_collision_demo.nlogo)

---

## Primitives:

#### Global:
[`phys:update`](#physupdate) [`phys:set-gravity`](#physset-gravity) [`phys:do-conservation`](#physdo-conservation)

---

#### Patch or Turtle Context:
[`phys:set-physical`](#physset-physical) [`phys:get-turtle-collisions`](#physget-turtle-collisions)

---

#### Turtle Context Only:
[`phys:push`](#physpush) [`phys:apply-force`](#physapply-force) [`phys:set-v`](#physset-v) [`phys:get-vx`](#physget-vx) [`phys:get-vy`](#physget-vy) [`phys:get-patch-collisions`](#physget-patch-collisions) [`phys:get-mass`](#physget-mass) [`phys:set-mass`](#physset-mass)

---
### phys:set-physical

 __`phys:set-physical `__  _`(boolean)`_ – _patch or turtle context_  

Tells the physics engine to make an object in its simulation world representing this turtle or patch in the NetLogo model if the parameter is true and if false remove it any such existing object. If the agent is a patch then the mass will be set to infinite making the object in the simulation immobile. If the agent is a turtle the mass will be set equivalent to the size of the turtle by default. The velocity and heading of the turtle associated object will be both be initially set to zero.

##### Examples:
```
; turns all white patches in the model into physical objects
ask patches with [pcolor = white] [phys:set-physical true]

; creates 3 turtles and makes them physical objects
create-turtles 3 [phys:set-physical true]
```


---
### phys:update

 __`phys:update`__  _`(change in time in milliseconds)`_ – _observer context_  

Tells the physics engine to simulate physics on all objects in its simulation world for an amount of time specified by the parameter and then modify the positions of the corresponding agents in the NetLogo model. During this period lists of collisions will be kept and corrections to velocity and positions of objects will be made to conserve energy if conservation is enabled.

##### Example:
```
; updates the world in the physics extension by one tenth of a second
phys:update 100
```

---

### phys:set-gravity

 __`phys:set-gravity`__ _`( x and y components of gravitational acceleration in m/s^2)`_ – _observer context_

Sets the gravitational acceleration in the simulation world. A positive value is down, a negative value is up.
##### Example:
```
; sets earth-like gravity in the physics simulation
phys:set-gravity 0 -9.81
```

---
 
 ### phys:set-v
 
__`phys:set-v`__ _`(velocity x and y components)`_ – _turtle context_ 

Sets the velocity of the object associated with the agent in the simulation world.

##### Example:
```
; swaps the x and y velocity of the agent
phys:set-v (phys:get-vy) (phys:get-vx)
```

---

### phys:get-vx

__`phys:get-vx`__ _(no parameters)_ – _turtle context_

Returns the x component of the velocity of the object in the simulation world associated with the agent.

##### Example:
```
; swaps the x and y velocity of the agent
phys:set-v (phys:get-vy) (phys:get-vx)
```
---

### phys:get-vy

 __`phys:get-vy`__ _(no parameters)_ – _turtle context_ 

Returns the y component of the velocity of the object in the simulation world associated with the agent.

##### Example:
```
; swaps the x and y velocity of the agent
phys:set-v (phys:get-vy) (phys:get-vx)
```

---
### phys:push

__`phys:push`__ _`(applied force in Newtons)`_ – _turtle context_

Apply a force to the object in the simulation world corresponding to the turtle in the the direction the turtle is currently heading.

##### Example:
```
; ask turtles to push with a random force with their current heading
ask turtles [phys:push (random 50)]
```

---
### phys:set-mass

__`phys:set-mass`__ _`(mass in kilograms)`_ – _turtle context_

Set the mass of the object in the simulation world corresponding to the turtle. The default mass is based on the size of the turtle.

##### Example:
```
; ask turtles to set their masses to a random value between 1 and 10.
ask turtles [phys:set-mass 1 + (random 10)]
```

---
### phys:get-mass

__`phys:get-mass`__ – _turtle context_

Returns the mass of the object in the simulation world corresponding to the turtle. The default mass is based on the size of the turtle.

##### Example:
```
; Show the mean mass of all turtles.
show mean [phys:get-mass] of turtles
```

---

### phys:apply-force

 __`phys:apply-force`__ _`(applied force in Newtons, direction of force)`_ – _turtle context_

Apply a force to the object in the simulation world corresponding to the turtle in the the direction specified by the second parameter.

##### Example:
```
; ask turtles to push 50 Newtowns in a random direction
ask turtles [phys:apply-force 50 (random 360)]
```

---

### phys:get-turtle-collisions

__`phys:get-turtle-collisions`__  _(no parameters)_ – _patch or turtle context_

Returns an agent set of turtles that the calling agent collided with the last time __phys:update__ was called.

##### Example:
```
; make all turtles that collided with yellow patches in the last turn become yellow
ask patches with [pcolor = yellow] [ask phys:get-turtle-collisions [set color yellow]]
```


---

### phys:get-patch-collisions

__`phys:get-patch-collisions`__ _(no parameters)_ – _turtle context_

Returns an agent set of patches that the calling turtle collided with the last time __phys:update__ was called. Patches cannot collide with other patches so this cannot be called from the patch context. 

##### Example:
```
; changes the color of patches to that of turtles that have collided with them
ask turtles [ask phys:get-patch-collisions [set pcolor [color] of myself]]
```

---

### phys:do-conservation

__`phys:do-conservation`__ _(boolean)_ - _observer context_

Tell the physics engine to make sure kinetic energy is perfectly conserved during collisions. If this is not enabled there may be be a slight gain in total systemic energy over time. Conservation does not function well when **gravity** is non-zero.

---
