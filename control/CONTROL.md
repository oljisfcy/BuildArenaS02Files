# Payload 30: Orbit and Return

The controller delivers a passive, native-weight-30 payload to orbit, releases it, then returns the carrier to the launch site. It combines scripted phase transitions with closed-loop guidance and attitude control.

Run the packaged machine with the official environment using [`reproduce.py`](reproduce.py). It starts the simulation, runs this controller, and checks the complete recorded trajectory. Use `--record` to set the camera to follow the payload Starting Block.

## Telemetry and outputs

At 25 Hz, full-profile BAT4 telemetry provides each block's position, quaternion, linear and angular velocity, and validity. Fuel, health and related resource fields are retained when available; missing values remain unknown.

The controller commands carrier-side Steering Thruster `fthrust` sliders and direction keys, Reaction Steering Block speed/damper sliders and keys, and the carrier Decoupler. Thruster sliders stay in `[0, 3]`. No payload block receives a control command.

## Mission phases

| Phase | Transition and control |
|---|---|
| **Ascent** | Velocity-error guidance with gravity/centripetal feedforward steers into the target orbital plane. Quaternion attitude feedback and angular-rate damping track the commanded thrust direction. |
| **Coast check** | A 2 s coast fits gravity from carrier motion. Release is allowed only with a valid fit, carrier periapsis above 1450, and payload anchor periapsis above 1300. |
| **Release and qualify** | Fire only the carrier-side Decoupler; use no escape thrust. Begin orbit qualification after payload/carrier anchor separation exceeds 50 and the payload is above radius 1100. |
| **Payload tracking** | Track the payload Starting Block on a fixed orbital plane. Require continuous progress of at least `6π + 0.01`, every payload part valid and above radius 1100, and the assembly intact. This continues during carrier recovery and after landing. |
| **Return and deorbit** | Predict the carrier's surface crossing and apply aerodynamic and burn lead. Burn retrograde until predicted periapsis is below 540, then shut off thrusters for the main glide. A nine-candidate range predictor selects pitch; bank guidance steers toward the original launch site. |
| **Powered landing** | Below height 600 on descent, use differential carrier thruster output for pitch/roll and reaction-wheel control for heading. Bounded position/velocity feedback brakes and hovers above the site, then lowers the height target at 0.6 units/s. |
| **Landed** | A low-speed/contact cue latches shutdown. All keys and sliders stay at zero. Continue recording until the orbit gate passes and the carrier meets the stability checks. |

## Tuned return parameters

- Orbit target radius 1950; ascent turn starts at radius 680. Radial ascent gain is 0.05, capped at 45; tangential target ramps from radius 650 over 900 units.
- Return periapsis target 540. Main-glide range guidance includes a ground-deceleration parameter of `1e6`, so it plans for negligible ground slide.
- Powered landing targets height 35. Horizontal target speed is `min(30, 0.35 × position error)`; horizontal gain is 0.9, with acceleration capped at 20. Vertical height and velocity gains are 0.25 and 1.
- Begin slow descent when horizontal error is below 0.4, horizontal speed below 0.3, and height is within 2 of 35. Pause descent if error exceeds 0.6. Latch shutdown near the original reference height when speed is below 1.5 and landing error below 0.7. Attitude slew is capped at 1.2 rad/s.

## Completion and checks

Mission completion requires three continuous payload revolutions, whole-payload clearance above radius 1100, positive conservative payload/carrier body clearance, and a stable carrier landing within 1 unit of the original launch target. The verifier checks the complete official CSV, all original block GUIDs, orbit progress, per-part bounds, separation, landing error, retained carrier pose and zero post-landing controls. Source binding is audited against the flown machine and controller.

This recovery uses a main glide followed by powered landing. 
