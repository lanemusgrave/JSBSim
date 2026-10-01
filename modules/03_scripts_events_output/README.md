# Module 03: Scripts, events and output

**Time:** about 3 h · **Reading:** RM ch. 4 *Scripting* (pp. 69–75); RM §3.4 *Initialization*; skim the upstream `scripts/` folder (e.g. `c1723.xml`, `c172_elevator_doublet.xml`)

## Objectives

1. Read and write a JSBSim `<runscript>`: `<use>`, `<run>`, local properties
   and `<event>`s with conditions, delays, `<set>` actions and `<notify>`.
2. Write an initialization file and know how scripts find it.
3. Log data with `<output>` and analyze the CSV with pandas.
4. Treat a script as a **flight-test card** and the CSV as the **flight-test
   data**: the same split you'll use at work.

---

## Script anatomy

```xml
<runscript name="...">
  <use aircraft="c172x" initialize="reset01"/>     <!-- aircraft/c172x/c172x.xml + reset01.xml -->
  <output name="log.csv" type="CSV" rate="20"> <property caption="kcas"> velocities/vc-kts </property> </output>
  <run start="0" end="90" dt="0.008333">
    <property value="0"> test/point </property>   <!-- a new, script-local property -->
    <event name="TP1" persistent="false" continuous="false">
      <condition logic="AND">                     <!-- one test per line; AND is the default -->
        simulation/sim-time-sec ge 10
        velocities/vc-kts gt 80
      </condition>
      <delay> 0.5 </delay>                        <!-- fire 0.5 s after the condition is met -->
      <set name="fcs/elevator-cmd-norm" value="-0.1" type="FG_DELTA"/>      <!-- add -0.1 -->
      <set name="fcs/throttle-cmd-norm" value="1.0" action="FG_RAMP" tc="2"/>
      <notify> <property caption="KCAS"> velocities/vc-kts </property> </notify>
    </event>
  </run>
</runscript>
```

| Element / attribute | Meaning |
|---|---|
| `<condition>` | Comparisons `lt le eq ge gt ne` (or `< <= == >= > !=`, escaped in XML). Nested `<condition logic="OR">` blocks are allowed. |
| *(default)* | Fires **once**, the first time the condition becomes true. |
| `persistent="true"` | Fires **every time** the condition goes false → true (watch items, limits). |
| `continuous="true"` | Executes **every step** while the condition is true (e.g. a `<set>` with a `<function>` making a sine wave; see the bundled `c172_elevator_doublet.xml`). |
| `type` | `FG_VALUE` (default: set to), `FG_DELTA` (add to), `FG_BOOL` |
| `action` | `FG_STEP` (default), `FG_RAMP` (linear over `tc` s), `FG_EXP` (first-order with time constant `tc`) |
| `simulation/do_simple_trim` | Setting it to a trim mode (0 longitudinal, 1 full, 2 ground…) trims the aircraft *at that moment*. |

**Initialization files** (`aircraft/<name>/<file>.xml`) set the ICs in one
place: `<vt unit="KTS">`, `<altitude unit="FT">`, `<latitude>`, `<psi>`,
`<gamma>`, `<running>` (engine index, or −1 for all). Scripts find them in the
aircraft's folder; from Python, `fdm.load_ic("reset01", True)`.

**Output** goes to CSV (`type="CSV"`), tab-separated text (`TABULAR`), UDP/TCP
(`SOCKET`; Module 16) or FlightGear's network protocol. `rate` is in Hz. You can also
use groups (`<rates> ON </rates>`, `<velocities> ON </velocities>`...) but
explicit `<property caption="...">` lists give clean column names.

## Walkthrough

1. Read the bundled `scripts/c1723.xml` (Module 00) again. You can now read
   every line.
2. Read [`solutions/c172_testcard.xml`](solutions/c172_testcard.xml), a
   four-point test card: elevator, aileron and rudder doublets and a power
   advance. Each doublet is built from three `FG_DELTA` steps, and there is a
   persistent "bank > 30°" watch item.
3. Run [`solutions/analyze_testcard.py`](solutions/analyze_testcard.py). It
   flies the card, reads the CSV with pandas, segments it by test point and
   prints a report:

   ```text
                          peak |q|  peak dNz  ...  peak |beta|  Dutch roll period [s]  climb rate [ft/min]
   TP1 elevator doublet       3.08      0.20
   TP2 aileron doublet                          ...
   TP3 rudder doublet                                    3.68                  2.87
   TP4 full power                                                                            1055
   ```

   Look at the plot (`outputs/03_scripts_events_output/testcard.png`) and notice:
   - **Re-trim is a teleport.** `do_simple_trim` resets attitude, altitude and
     rates, so the states *jump* at 25, 45 and 65 s. That makes it a useful
     simulation tool, but those samples are not flyable data, so cut them out
     before you analyze anything (the segment windows do).
   - The 2.9 s Dutch roll period from zero crossings of r. Module 06 will
     predict it from the linear model.
   - The "climb rate" in TP4 is partly a **zoom**: the airplane trades its
     initial speed for altitude during the phugoid. A 10 s window is too short
     for a steady-state climb number, which is the same lesson as a real
     sawtooth climb card.

### Gotchas

- **Two dashes are illegal inside an XML comment.** Writing `--script` inside
  `<!-- -->` breaks the whole file with "not well-formed".
- **Script-local properties can't be logged by the script's own `<output>`**
  (the output block is read before `<run>` declares them). JSBSim only warns:
  *"No property by the name test/point has been defined. This property will
  not be logged."* Read warnings!
- Output file paths are relative to the output directory: `outputs/` with
  `gnclab`, or the JSBSim root folder with the bare CLI.
- The CSV is flushed when the executive is destroyed (`del fdm`) or the
  program exits.

---

## Exercises

1. **[`exercises/my_card.xml`](exercises/my_card.xml):** complete the card so
   the C172's own autopilot holds altitude, turns 90° and then climbs 500 ft.
   Run it with
   `python modules/03_scripts_events_output/solutions/run_card.py modules/03_scripts_events_output/exercises/my_card.xml --show`.
   Compare with [`solutions/c172_ap_card.xml`](solutions/c172_ap_card.xml).
2. Add a **3-2-1-1** elevator input to `c172_testcard.xml` at t = 80 s
   (unit width 0.4 s, amplitude 0.05), using `FG_DELTA` steps. You'll use
   this maneuver for system ID in Module 14.
3. (Optional, rocket GNC case study.) Run the bundled `scripts/J2460.xml`
   (J-246 two-stage rocket) with `run_card.py`-style Python. List its events
   and explain the staging logic from the XML.

## Self-check

- What's the difference between a persistent event and a continuous event?
- How would you make the throttle go from 0.6 to 1.0 smoothly over 3 s?
- Why must a test card re-establish the test condition between points, and
  why is `do_simple_trim` not the same as a pilot doing it?
- Your script's CSV has a column of zeros for a property you asked for. What
  happened?

## Done when

- [ ] Your `my_card.xml` flies and logs the turn and the climb
- [ ] You can write an event with a multi-line condition, a delay and a ramp from memory
