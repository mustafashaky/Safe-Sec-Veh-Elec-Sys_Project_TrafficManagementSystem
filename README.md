Directory: traffic_sim_Motor_Knights >> Ahmer_traffic_sim >> 
1st draft- Rule based traffic_sim


Directory: traffic_sim_Motor_Knights >> Mustafa_traffic_sim >> 
Primary files: play_dqn_traffic_sim_2_smart , rl_env_traffic_sim_2_smart , traffic_sim_2_smart , train_dqn_traffic_sim_2_smart


Directory: traffic_sim_Motor_Knights >> trial1_traffic_sim >> 
trial1_traffic_sim files: dqn_phase.pt , play_dqn_traffic_sim_2_smart , Read Me , rl_env_traffic_sim_2_smart , traffic_sim_2_smart , train_dqn_traffic_sim_2_smart

"Traffic Management System Trial 1"

1. traffic_sim_2_smart.py  (the live traffic sandbox)

A small visual program that opens a window with a four‑way intersection. It draws roads and cars and runs a “smart” traffic light.
It figures out what each car will really do (go straight, turn right, turn left) from the path the car is on, even if the car “claims” something else.
If a car lies, it receives a short-term penalty (a red dot appears above it while it waits).
Only the front car in each lane is allowed to go on green, so lines move naturally.
If there’s a crash, the system frees that lane so it doesn’t get stuck forever.

Blue cars are Honest, Red cars are Liars, and Yellow cars have Crashed.
The little 3‑box indicators near each approach show which movements are Green/Red.
On‑screen text reminds you of controls and what colors mean.

Run: python traffic_sim_2_smart.py
Keep the Pygame window focused. Press 1 to spawn a safe demo (four honest right turns). Press 2 to spawn a liar demo (the East car lies; it gets the red dot and waits longer).
Watch honest cars flow on green. Lying cars sit with the red dot and go later.


2. rl_env_traffic_sim_2_smart.py  (the “training arena” (no window))

A behind‑the‑scenes version of the simulator that runs without graphics so the computer can practice controlling the lights quickly.

The arena gives the learner a “state” (simple numbers like how many cars are waiting, which phase is active, how much time is left in that phase).
The learner picks an “action” (which phase to show next, and for how long).
The arena returns a “reward” score each step: lower delays are better, crashes are heavily penalized, and very unfair waiting gets a small penalty.

This lets a simple AI try many options and learn a decent timing strategy.

You normally don’t run this directly. The training script (next file) uses it automatically.


3. train_dqn_traffic_sim_2_smart.py  (the teacher that creates a model)

A script that uses the training arena to teach a small model to pick traffic light phases and durations.

Runs lots of short practice rounds, sees what works (less waiting, no crashes), and gradually improves.
Saves the learned model as a file named dqn_phase.pt in your project folder (the path is set near the bottom of the file; you can change it).

Lines like “ep 042 steps 0600 reward …” printing progress. As it learns, rewards should usually get less negative (better).
At the end: “Saved model to: D:/…/dqn_phase.pt”.

Run: python train_dqn_traffic_sim_2_smart.py
Wait until it finishes and confirms where it saved dqn_phase.pt.


4. play_dqn_traffic_sim_2_smart.py  (the live demo using the learned model)

A visual run of the intersection where the learned model decides which phase is green and for how long. The “smart guardrails” still apply (intent inference, liar penalty, front‑of‑lane release).

At the top of this file, MODEL_PATH must point to your saved dqn_phase.pt. You can change that path if you saved the file somewhere else.

Same window as the sandbox, but now the phase changes follow the learned model.
Honest cars should pass smoothly on green. Lying cars show the red dot and are delayed, then pass when it’s safe.

Run: python play_dqn_traffic_sim_2_smart.py
Keep the window focused. Press 1 to spawn the safe demo, 2 to spawn the liar demo. Watch how the learned controller handles it.


Instruction to use the modules: 

Step 1: 
Explore the sandbox (optional but helpful).
python traffic_sim_2_smart.py.
Get familiar with the visuals and hotkeys (1 and 2).
Step 2: 
Train a model.
python train_dqn_traffic_sim_2_smart.py.
When done, confirm dqn_phase.pt exists at the path shown.
Step 3: 
Play with the learned controller.
Make sure play_dqn_traffic_sim_2_smart.py has the correct MODEL_PATH.
python play_dqn_traffic_sim_2_smart.py.
Press 1 and 2 to see how it behaves.

To tweak behavior:

LIAR_PROBABILITY (traffic_sim_2_smart.py): make lying rarer or more common.
SPAWN_INTERVAL: smaller = heavier traffic, larger = lighter traffic.
LIE_PENALTY_SEC: increase or decrease how long liars wait before they’re eligible to go.
PHASE_DURATION (only used in the sandbox’s internal greedy choice; the RL player ignores it and uses its own durations).
