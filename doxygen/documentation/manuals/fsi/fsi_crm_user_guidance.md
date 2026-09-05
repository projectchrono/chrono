Chrono::CRM Practical Guidance {#manual_fsi_crm_user_guidance}
==============================

\tableofcontents

Who this page is for
--------------------

This page is for someone using Chrono's continuum soil model, CRM, for the first time or the first
few times. CRM represents soil as a cloud of particles and solves the soil equations on them, a
method called SPH (Smoothed Particle Hydrodynamics). The page collects the settings and checks that
are easy to get wrong and that the code will not warn you about. Where a setting matters, we say how
much, and we say under what conditions we measured it, because most of these numbers do not transfer
to other conditions.

CRM is a strong tool. It is fast on a GPU and, for the things it is good at, it is very good. Nothing
here is a reason not to use it. It is a list of things to check so that the number you get out is the
number you think you got.

Read this page after the parameter guide, @ref manual_fsi_sph_parameter_selection, and the rheology
pages, @ref manual_fsi_mu_i_rheology and @ref manual_fsi_mcc_rheology.

Five things most likely to catch you by surprise, in the current implementation:
-----------------------------------

1. **The friction coefficient is not a friction angle.** Setting a value that looks like a 21 degree
   soil does not give you a 21 degree soil. See the section on friction below.
2. **The rate-dependent part of the mu(I) model is switched off unless you switch it on.** Every
   shipped demo and test that sets the two friction coefficients sets them equal (all 24 of them on
   today's Chrono), and so do the library defaults. If you did not deliberately give the two
   coefficients different values, you ran a constant-friction model under the name mu(I). How much
   that matters depends on how fast the soil moves; the table in the mu(I) section gives the measured
   cases.
3. **The free-surface treatment has a switch-on value; check whether yours is on.** When it is on,
   it changes the bearing pressure under a plate by about a third, so know which side of the switch
   you are on.
4. **An active domain that is too small changes your answer, not only your run time.** Double the
   box once and rerun; if the result moves, the box was too small.
5. **Let the soil settle until it stops moving.** A bed that is still moving when you load it, or
   drive a vehicle onto it, produces a wrong answer that looks like an ordinary one.

Good habits
-----------

**Settle until the bed is still, not until a clock says so.** Decide stillness with a bulk statistic,
such as the speed that 99 percent of particles are below, never with the fastest particle (the next
habit says why). Choose the cutoff small compared with how fast you will load the soil, and record it
with the result. Give the settling stage a maximum time as well, and make your script print which of
the two it hit. A bed that has not
finished settling looks exactly like one that has, right up to the moment the numbers come out wrong.
A run that gave up on settling and carried on still prints a result, and that result sits in a table
next to good ones looking like ordinary scatter.

**Do not use the fastest particle as a health check.** It is the natural thing to look at and it will
mislead you. Particles at a free surface can be thrown off at speeds hundreds of times the speed of
whatever is driving the soil, while the bulk of the bed is barely moving. In one case a plate pushed
at 10 mm/s produced a peak particle speed of 5.4 m/s, while 99 percent of the 196,599 particles were
moving slower than 0.011 m/s. About 20 particles were anywhere near that peak. Use a high percentile
(the speed that 99 percent of particles are below), or count the particles above a threshold, and
keep the maximum as something you glance at rather than something you decide by.

The same trap can flip a conclusion. In one pair of runs, a bed of about a million particles under
a wheel at 90 percent slip, the fastest particle averaged 3.77 m/s over the run with the rate term of
the mu(I) model off and 4.03 m/s with it on. Read naively, engaging the rate term made the bed more
violent. In fact the "fastest particle" was a single particle out of a million in almost every frame,
and in the frame we examined the speed that 99 percent of the bed sat below went the other way: 0.0100
m/s with the rate term off, 0.0047 m/s with it on. That is what a model that raises friction when
soil moves fast should do. A statistic that reverses the sign of your conclusion is worse than one
that is merely noisy.

**Compare results at the same displacement, not the same time.** Two runs that stopped at whatever
sinkage they happened to reach cannot be compared to each other or to an experiment.

**Record which Chrono build produced each result.** If your script re-runs CMake, check afterwards
which library it actually linked. A build configuration that changes silently produces results that
look valid and are not.

The settings that matter
------------------------

### The friction coefficient is not a friction angle

In soil mechanics you are used to describing a soil by its friction angle, say 30 degrees. CRM does
not take a friction angle. It takes a friction coefficient, a plain number with no units. On this page
we call it `mu_s`; in the code it is the field `mu_fric_s` of the soil properties.

The mu(I) parameter guide (@ref manual_fsi_mu_i_rheology) offers `mu_s = tan(angle)` as a starting
point, so that a 30 degree soil becomes `mu_s = 0.58`. Treat that as a first guess, not as a
conversion. The formula holds for a block sliding on a plane. Inside a soil the stress state is
three-dimensional. There is more than one accepted way to relate a coefficient like `mu_s` to a
friction angle, and the two most common conventions give answers that differ by more than a factor
of two for the same `mu_s`. Chrono's documentation does not say which convention its `mu_s` follows,
so a converted number has no fixed meaning.

What you can trust is a measurement. In one test we let a column of CRM soil collapse into a pile and
measured the slope of the pile, which is the classic way to read off a friction angle from a real
material. For `mu_s = 0.38`, which the formula calls a 21 degree soil, the pile stood at about 11
degrees, measured two ways (the steepest local slope, and the pile height over its spread). That
number is for that column test only. It is not a conversion factor, and it does not transfer to a
plate or a wheel.

The practical advice: pick `mu_s` by matching a measurement you care about (a pile angle, a plate
test, a wheel test), not by converting a friction angle. If you measure a pile yourself, read its
angle from the steepest part of the slope or from the pile height over its spread. Do not fit a
straight line across the whole side: at high friction the pile becomes a rounded dome, and a
straight-line fit understates the angle.

### The mu(I) rate term is off unless you turn it on

The mu(I) model makes friction depend on how fast the soil is moving relative to how hard it is
pressed together, through a dimensionless number called the inertial number `I`:

    mu = mu_s + (mu_2 - mu_s) * I / (I0 + I)

Here `mu_s` is the friction at rest (code field `mu_fric_s`), `mu_2` is the friction at very high
rates (code field `mu_fric_2`), and `I0` is a reference value of the inertial number that sets how
quickly friction rises between the two (code field `mu_I0`). If you choose `mu_2` equal to `mu_s`,
the second term is zero for every `I` and the model is plain constant friction. Every shipped demo
and test that sets both coefficients sets them equal, and so do the library defaults. This is
documented behaviour, not a bug. But it means a result labelled mu(I) is a constant-friction result
unless the two coefficients were deliberately set apart.

How much this matters depends on how fast the soil moves. These are the cases we measured:

| case | how fast the soil moves | effect of turning the rate term on |
|---|---|---|
| plate pushed into the soil at 5 to 10 mm/s | slowly | nothing measurable (13 repeats) |
| the same plate pushed at 50 to 1500 mm/s | quickly | +26 to +81 percent of the reaction force, largest at 500 mm/s and not steadily increasing with speed |
| wheel at 30, 60, 90 percent slip, 10 mm spacing | driven | +8.6, +11.5, +18.1 percent of drawbar pull (the net forward force the wheel produces) |
| collapsing column of soil | free flow | same final pile as a 57 percent higher friction would give |

The reason: the inertial number is largest where soil moves fast under little pressure, which is the
thin skirt racing outward at the foot of a collapsing pile, not the slow core. So the rate term
steepens the outside of a deposit and leaves the inside alone.

If you are modelling anything that flows, give `mu_fric_2` a value above `mu_fric_s`. The mu(I)
parameter guide, not this page, is where to pick the values.

### Check whether the free-surface treatment is actually on

Particles at the open top of the soil are treated specially: their stress is reset so the surface
cannot carry tension it should not. Which particles count as "at the surface" is decided by the
parameter `free_surface_threshold`, compared against a measure of how many neighbours a particle
has on all sides (`nabla_r`, close to 3 deep in the bed and lower at the surface).

The point to know is that the treatment has a switch-on value, and small changes around it matter.
In a settled bed at 12.5 mm particle spacing we measured `nabla_r` at about 2.9 deep inside and about
2.01 in the top layer (the same values came back at 25 mm spacing). A particle is flagged when its
`nabla_r` is below the threshold. So a threshold below 2.01 flags no particle at all, and the treatment
is off; the library default is 2.0. A threshold just above 2.01 flags the top layer only. Raising it
further flags more and more layers, up to the whole bed near 2.9, so there is no single "on" state.
Whether you want the treatment on, and how deep, is a modelling choice. What you should not do is
assume it is on because the parameter exists. In our plate tests (12.5 mm spacing, single precision,
compared at 9.5 mm of sinkage) a threshold of 2.2 lowered the bearing pressure by about a third and a
threshold of 2.6 by about 45 percent, so this is not a fine-tuning knob.

How to check in your own run: Chrono does not write the free-surface flag to its output files, so
you cannot count flagged particles from the output. What you can do is find out whether the treatment
affects the quantity you care about. Run your case once at your threshold and once at a very low
threshold such as 0.8. In a packed bed no particle sits far below 2.01, so 0.8 flags at most a few
nearly isolated particles, such as grains thrown clear of the surface; it is a comparison point, not a
guaranteed off switch. If the two runs differ, the threshold matters for your quantity and you have a
modelling choice to make. If they agree, you have learned only that those two thresholds give the same
answer for that quantity. The parameter guide does not yet say what the default is meant to do.

### The active domain: what it freezes, and what it changes

To save time on a large problem, Chrono can simulate the soil only inside a box around a moving body
and leave everything outside that box frozen. This is called an active domain. How to switch it on is
described in @ref vehicle_terrain_crm_performance; note that the example on that page also sets
`num_proximity_search_steps = 10`, which item 4 under Known limitations asks you to check before
copying. An active domain is a legitimate speed-up, and it has two consequences that are easy to
miss.

**First, most of the stress field in your output is not current.** A frozen particle keeps whatever
stress it had at the moment it was last updated. In a wheel run over a bed of a million particles,
almost the whole bed shows exactly zero pressure at the start, and as the wheel advances the region
with non-zero stress grows and keeps growing, stretching from the wheel back to where it started. Nothing marks stale values as stale. So at any instant the output holds three populations:
live values near the body, stale values in the wake, and exact zeros ahead of the body in soil that
has never been active. Any statistic over the whole bed mixes all three. **If you use an active
domain, post-process only inside it, or turn it off for the run you intend to analyze.**

**Second, the box changes the forces if it is too small.** An active domain is only a free speed-up
when it is large enough that making it larger changes nothing. Soil the body is still shearing must
be inside the box. The check is simple and costs one extra run: double the box, rerun, and compare
the quantity you care about. If it moves, the box was too small. If it does not, you have earned the
speed-up.

We ran that check on the Viper wheel test rig at 40 percent slip, 10 mm spacing, single precision.
The rig's active box is an axis-aligned box centred on the wheel hub and moving with it, given by its
minimum and maximum corners in metres relative to the hub. The box the rig estimated for itself
extended 1.25 wheel radii forward and backward, 0.625 wheel widths to each side, and 1.25 wheel radii
up and down, and it was too small: the drawbar pull converged to about 252 N once every one of those
extents was doubled (2.5 radii, 1.25 widths, 2.5 radii), and stayed there at four and eight times the
estimate, while the estimate gave a lower and fluctuating force. The 252 N is a number for that test,
not a target for the Viper wheel. The rig's estimate is being enlarged in Chrono to the doubled box; if
your Chrono predates that change, pass the rig a box of your own or run the doubling check.

Two further observations from the same tests, so you know what to expect:

1. How much a too-small box costs depends on the operating point. The same wheel at 60 percent slip
   showed almost no force change from doubling the box, but the measured effect of the mu(I) rate
   term doubled (from about 6 to about 12 percent). A box that is too small can hide a parameter's
   influence while the force looks fine, which is the harder mistake to notice.
2. A converged box costs real time (about 2.8 times a small one in our case). That is exactly why a
   too-small box is easy to keep and hard to notice, and why the doubling check is worth one run.

At very large particle counts there is a limit we have not resolved: in a 12.5-million-particle case
the doubled box made the run stop within seconds, because a handful of soil particles were thrown
far outside the bin and Chrono's neighbour search halts on that by design, while the same case ran
to completion with the small box. Until this is understood, treat the doubling check as reliable up
to a few million particles. If the doubling run stops on you, do not fall back on the small-box
force: it is the very number the check could not confirm. Turn the active domain off for the run you
will report, or enlarge the soil bin so the doubled box fits inside it.

**What to do:**

1. On the wheel test rig, with a Chrono that predates the enlarged estimate, pass
   `SetWheelActiveDomain` a box of your own. The argument is a `ChAABB`, the box's minimum and maximum
   corners in metres relative to the wheel hub, x forward, y to the side, z up. For the Viper wheel
   (radius 0.245 m, width 0.2 m) the box that converged in our test was `ChAABB(ChVector3d(-0.6125,
   -0.25, -0.6125), ChVector3d(0.6125, 0.25, 0.6125))`, that is 2.5 radii forward and backward, 1.25
   widths to each side and 2.5 radii up and down; four and eight times the old estimate gave the same
   answer. Scale the corners with your own wheel's radius and width.
2. On any other problem, run the check yourself: double the box, rerun, and compare the quantity you
   care about. If it moves, the box was too small, and one doubling may not be enough; double again
   until it stops moving. If it does not move, you have earned the speed-up. The check costs one extra
   run. A converged box cost us about 2.8 times a small one (4 seconds of simulation, 205,821
   particles, one GPU), which is exactly why a too-small box is easy to keep and hard to notice.

### Density means different things under the two rheologies

Under mu(I), Chrono resets density to the reference value every step and takes pressure from the
stress tensor. Under Modified Cam-Clay (MCC), density is integrated normally. So a density that varies
with depth under MCC will be uniformly the reference value under mu(I), even under heavy load. If your
post-processing reads density as a state variable, it is meaningful under one rheology and meaningless
under the other.

Checks you can run in an afternoon
----------------------------------

### The geostatic check, and the good news it brings

Settle a cohesionless bed until it is still and check that the vertical stress follows the textbook
law for soil under its own weight, `sigma_v = gamma * d`, with `gamma` the unit weight and `d` the
depth. **CRM passes this cleanly, which is the single most important thing to know about it.** A fresh
bed with zero cohesion reproduced the law at every spacing and column height we tried:

| spacing | column height | particles | measured slope divided by gamma |
|---|---|---|---|
| 0.02 m | 0.9 m | 106 thousand | 0.991 |
| 0.01 m | 0.9 m | 872 thousand | 1.002 (identical in single and double precision) |
| 0.005 m | 0.9 m | 7.09 million | 1.002 |
| 0.02 m | 3 m | 3.3 million | 1.008 |

Refining the resolution does not degrade this. A flaw in how the equations are approximated on the
particles would show the opposite trend. If this check fails for you, look at your usage before
suspecting the solver. The two causes we have met: a bed that had not finished settling (too short a
settle, or too little artificial viscosity, the numerical damping the solver adds to calm a bed), which
gets worse for taller columns; and a bed saved by an older Chrono with a different cohesion treatment.
The habits above avoid both, and this check is how you catch either one.

### The precision check

Run one case you care about in single and double precision and overlay the two load curves. Two lines
on top of each other tell you single precision is fine for your problem. A steady offset tells you it
is not. The next section says what to expect.

### The settle honesty check

Log the velocity statistic your settling criterion uses, at every check, and make the run state
plainly whether it converged or hit its time limit. If your script cannot tell those two apart in its
output, fix that before you trust anything it produces.

Single or double precision
--------------------------

**For quiet, low-stress work, single precision is free.** On a settled bed under its own weight,
single and double precision agreed to four significant figures in our geostatic tests. Use single and
take the speed.

**For failure loads, prefer double precision, especially with mu(I).** In our plate tests (a plate
pushed into the soil until it fails), single precision under-predicted the failure load by roughly 1
to 4 percent with Modified Cam-Clay and by a steadier 6 to 9 percent with mu(I). Run-to-run noise in
those tests was under 1 percent, so this is a real effect.

**It is not overflow.** Peak stresses in those tests were about 20 MPa, far below what a 32-bit float
can hold. The likely cause, which we have not proven, is accumulated rounding in the step that
decides whether the soil yields, to which mu(I) appears the more sensitive.

So: if your deliverable is a failure load such as a bearing capacity, run in double precision. We
measured this on a plate; for a wheel's drawbar pull or a sinkage resistance we have not repeated the
comparison, so run the precision check above once yourself. If your deliverable is a settling study
or a quiet stress field, single precision is fine and faster.

Soil leaving the surface at high speed
--------------------------------------

Soil can leave a free surface at speeds far above anything driving the simulation. We measured
particles leaving at about 5.4 m/s from a bed being loaded by a plate at 10 mm/s. The ejection speed
barely follows the driving speed: across an eightfold increase in plate speed it changed by a factor of
1.36, and not steadily. That weak, uneven dependence does not look like soil being squeezed out by the
plate, which would follow the plate's speed; we have not established what it is. Whether it comes from
the particle method in general or from Chrono's implementation of it we have not tested. It matters mainly for diagnostics, as described
under good habits: it is invisible in bulk statistics and dominates any maximum.

Known limitations
-----------------

1. **No checkpoint or restart in FSI-SPH.** A long CRM run cannot be paused and resumed faithfully.
   In particular, the Modified Cam-Clay model tracks how much the soil has compacted over its history,
   and that history cannot be rebuilt from a saved stress state. Plan runs to fit in one sitting.
2. **Set every parameter before `Initialize()`.** The solver copies its parameters to the GPU once, at
   initialization. Newer Chrono versions throw an error if you change a guarded parameter afterwards;
   older ones ignore the change silently. Check which you have.
3. **A moving computational domain may translate but not resize** after initialisation. Changing its
   extent trips an assertion. That is a reasonable design line, but plan for it.
4. **If you raise `num_proximity_search_steps` above 1 to save time, check once against 1.** This
   setting rebuilds the list of neighbouring particles only every N steps. The performance page
   linked above shows a value of 10 in its example. In Chrono versions before the fix for issue #823
   (the fix refreshes the wall velocity the soil sees on the steps that skip the rebuild), any value
   above 1 corrupted that velocity for a rigid body in the soil: a plate pushed at 10 mm/s read 95 kPa
   where it should read 174. With the fix, a value of 8 agreed with 1 within 1.4 percent on a moving
   plate and saved about 17 percent of run time. One comparison run against 1 tells you which Chrono
   you have.

Acknowledgments
---------------

This page was prepared by the Simulation-Based Engineering Laboratory, University of
Wisconsin-Madison.

This work used computing resources made available through the AMD University Program (AUP) AI & HPC
Cluster. Additional runs used NVIDIA hardware at the Simulation-Based Engineering Laboratory,
University of Wisconsin-Madison.
