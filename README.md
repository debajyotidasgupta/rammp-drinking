# rammp_drinking

Robot-assisted drinking on the Kinova Gen3, as a RAMMP module: it picks the user's cup up by its handle, brings it to
their open mouth, holds it there for a sip, and sets it back where it stood. It is the drinking skill of the earlier
RAMMP system (empriselab/RAMMP: pick_drink, transfer_drink, stow_drink), on the December platform.

It runs as a capability engine (rammp-kitchen-capability-engines' `CapabilityEngine`, engine id `drinking`): the
orchestrator starts its actions, `pick_up_drink`, `give_sip` (one sip) and `put_down_drink`, and nothing moves otherwise
([interface](docs/interface.md)). The `rammp-drink` command runs the same steps by hand, for setting up a site.

It is built on rammp-feeding's image and uses its code as it is: the guarded arm (every move checked as it runs), the
shared planner's paths checked for what the gripper holds, the wrist camera, MediaPipe's face and the presentation at
the mouth. It shares feeding's state folder: one site file, one table, one wrist camera mount, one home pose. Drinking
needs the gripper empty: the fork comes out first (by hand, or by rammp-feeding's put_down_fork where it hangs on a
stand) and goes back after (pick_up_fork).

**Release 0.1.2:** `debajyotidasgupta/rammp-drinking:0.1.2` (Docker Hub, arm64), on `rammp-feeding:0.6.1`, for the arm
stack's 2.0 generation (kinova-gen3-ros2 2.0.0, rammp-curobo 2.0.1).

## How a drink goes
1. **Pick up:** from `drink_gaze`, SAM3 finds the cup's handle in the wrist camera and its depth gives the handle's
   frame and the opening inside it. The gripper, fingers closed, goes in front of the opening, into it and up under its
   top, and opens slowly: the fingers hold the handle from inside (opened all the way, they missed it: it backs out).
   It lifts the cup and carries it, upright, to `drink_hold`.
2. **Sip:** it carries the cup to `drink_deliver`, waits for an open mouth, measures the mouth once, and moves the cup's
   tip straight to `delivery.stop_m` (0.10 m) short of it. It holds the cup there `drinking.sip_s` (10 s), then backs
   away and returns to `drink_hold` along the way it came.
3. **Put down:** it carries the cup back over where it stood, sets it down, closes the fingers (letting go), backs out of
   the handle and goes home, each path the pick's in reverse.

Every path that carries the cup is the shared planner's `HOLD_LEVEL` path (its tilt held all the way) round the arm's
base the short way, and we check it again for the cup's tilt and its shape against the table and the user before it
runs. Every way back retraces its way out, so no joint winds up (the wrist camera's cable).

## Getting started
On a robot already set up for rammp-feeding:
1. **Site file:** add the `drinking:` section of [`config/machines/template.yaml`](config/machines/template.yaml) to
   `/srv/rammp_feeding/config/local.yaml`. Measure the cup with a ruler, and set `workspace.table_height_m`, which
   feeding uses too.
2. **Poses:** take the fork out of the gripper. Then `rammp-drink capture drink_gaze`, `drink_hold` and `drink_deliver`
   (place the cup in the gripper by hand for the last two: `rammp-drink gripper close`, the handle over the fingers,
   `rammp-drink gripper open`).
3. **Check:** `rammp-drink check`. Then `rammp-drink pick`, `sip` and `put-down` by hand; under the orchestrator, the
   `drinking` node's actions.

The [deployment checklist](docs/porting.md) has the details.

## Layout
```
rammp_drinking/  engine.py (the node)  cli.py (the rammp-drink command)  actions.py (pick_up, give_sip, put_down)
                 handle.py (the handle and the grasp)  mug.py (the cup and the planner's world)  config.py
config/          drinking.yaml (every tuned value), machines/template.yaml
launch/  tests/
```

## Build and test
Tests run on an arm64 Docker host (a Jetson), not on a laptop:
```bash
docker build --build-arg FEEDING_IMAGE=debajyotidasgupta/rammp-feeding:0.6.1 -t rammp_drinking:dev .
docker run --rm -v "$PWD:/work" -w /work rammp_drinking:dev bash tests/offline.sh   # unit tests + the whole drink (fake robot)
ROBOT=<rammp-deployments config/robots/<robot>> tests/test_e2e.sh                   # the node's actions through the real 2.0
                                                                                     # driver, description and planner (a CUDA
                                                                                     # host); then the node as sheppy starts it
```
Both run on two test sites (`tests/fixture.py`): the arm level on its mount (the default) and `RAMMP_TEST_SITE=tilt30`,
rammp-feeding's arm leaning 30 deg (for the end to end, a robot folder of such an arm).
To release: build `debajyotidasgupta/rammp-drinking:<version>` from the tagged commit on the feeding image it names,
push it, and pin that tag in the manifest.
