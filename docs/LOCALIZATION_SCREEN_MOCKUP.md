# Localization Screen Mockup

<div class="page-intro">
This is a docs-side mockup of the runtime localization page direction in <code>src/ui/display.cpp</code>. It reflects the current field model in <code>include/config.h</code>: north and south long-goal supports centered at <code>x = +/-24 in</code>, <code>y = +/-48 in</code>, the center <code>X</code> goal as two intersecting diagonal bars, one loader square in each quadrant at the field edge, robot footprint sizing from config, and distance-sensor sightlines for <code>PureMcl</code> and <code>Combined</code>.
</div>

<div class="brain-mockup">
  <div class="brain-mockup__header">
    <div class="brain-mockup__brand">69580A Brain Screen</div>
    <div class="brain-mockup__tabs">
      <span class="brain-tab">SELECT</span>
      <span class="brain-tab brain-tab--active">ODOM</span>
      <span class="brain-tab">PID</span>
      <span class="brain-tab">PATH</span>
      <span class="brain-tab">GPS</span>
    </div>
  </div>
  <div class="brain-mockup__body">
    <section class="brain-panel">
      <div class="brain-panel__title">
        <span>LOCALIZATION MAP</span>
        <span class="brain-view-pill">COMBINED</span>
      </div>
      <svg class="brain-field" viewBox="0 0 360 360" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="Localization minimap mockup">
        <rect x="20" y="20" width="320" height="320" rx="12" class="brain-field__mat"/>

        <g class="brain-field__grid">
          <line x1="73.3" y1="20" x2="73.3" y2="340"/>
          <line x1="126.6" y1="20" x2="126.6" y2="340"/>
          <line x1="180" y1="20" x2="180" y2="340"/>
          <line x1="233.3" y1="20" x2="233.3" y2="340"/>
          <line x1="286.6" y1="20" x2="286.6" y2="340"/>
          <line x1="20" y1="73.3" x2="340" y2="73.3"/>
          <line x1="20" y1="126.6" x2="340" y2="126.6"/>
          <line x1="20" y1="180" x2="340" y2="180"/>
          <line x1="20" y1="233.3" x2="340" y2="233.3"/>
          <line x1="20" y1="286.6" x2="340" y2="286.6"/>
        </g>

        <g class="brain-field__corners">
          <rect x="20" y="20" width="53.3" height="53.3" rx="8" class="brain-field__corner brain-field__corner--red"/>
          <rect x="20" y="286.7" width="53.3" height="53.3" rx="8" class="brain-field__corner brain-field__corner--red"/>
          <rect x="286.7" y="20" width="53.3" height="53.3" rx="8" class="brain-field__corner brain-field__corner--blue"/>
          <rect x="286.7" y="286.7" width="53.3" height="53.3" rx="8" class="brain-field__corner brain-field__corner--blue"/>
        </g>

        <g class="brain-field__obstacles">
          <rect x="121.0" y="67.0" width="8.6" height="7.2" rx="2" class="brain-field__obstacle"/>
          <rect x="230.4" y="67.0" width="8.6" height="7.2" rx="2" class="brain-field__obstacle"/>
          <rect x="121.0" y="285.8" width="8.6" height="7.2" rx="2" class="brain-field__obstacle"/>
          <rect x="230.4" y="285.8" width="8.6" height="7.2" rx="2" class="brain-field__obstacle"/>

          <rect x="20.0" y="67.6" width="5.9" height="5.9" rx="1.5" class="brain-field__obstacle"/>
          <rect x="20.0" y="286.5" width="5.9" height="5.9" rx="1.5" class="brain-field__obstacle"/>

          <rect x="334.1" y="67.6" width="5.9" height="5.9" rx="1.5" class="brain-field__obstacle"/>
          <rect x="334.1" y="286.5" width="5.9" height="5.9" rx="1.5" class="brain-field__obstacle"/>

          <rect x="158" y="175" width="44" height="10" rx="3" transform="rotate(45 180 180)" class="brain-field__obstacle"/>
          <rect x="158" y="175" width="44" height="10" rx="3" transform="rotate(-45 180 180)" class="brain-field__obstacle"/>
        </g>

        <g class="brain-field__imu">
          <circle cx="43" cy="43" r="3"/>
          <line x1="43" y1="43" x2="55" y2="43"/>
        </g>

        <g transform="translate(244 238) rotate(-28)">
          <rect x="-20" y="-14" width="40" height="28" rx="4" class="brain-field__robot"/>
          <line x1="0" y1="0" x2="20" y2="0" class="brain-field__robot-nose"/>
          <circle cx="0" cy="0" r="2.5" class="brain-field__robot-center"/>

          <circle cx="-13" cy="-8" r="1.8" class="brain-field__sensor"/>
          <circle cx="-13" cy="8" r="1.8" class="brain-field__sensor"/>
          <circle cx="15" cy="0" r="1.8" class="brain-field__sensor"/>
          <circle cx="-17" cy="0" r="1.8" class="brain-field__sensor"/>

          <line x1="15" y1="0" x2="72" y2="0" class="brain-field__ray"/>
          <circle cx="72" cy="0" r="2.5" class="brain-field__hit"/>

          <line x1="-13" y1="-8" x2="-13" y2="-52" class="brain-field__ray brain-field__ray--ghost"/>
          <line x1="-13" y1="8" x2="-13" y2="52" class="brain-field__ray brain-field__ray--ghost"/>
          <line x1="-17" y1="0" x2="-49" y2="0" class="brain-field__ray brain-field__ray--ghost"/>
        </g>
      </svg>
      <div class="brain-legend">
        <span class="brain-legend__item"><span class="brain-legend__swatch brain-legend__swatch--obstacle"></span>Obstacle surfaces</span>
        <span class="brain-legend__item"><span class="brain-legend__swatch brain-legend__swatch--robot"></span>Robot footprint</span>
        <span class="brain-legend__item"><span class="brain-legend__swatch brain-legend__swatch--ray"></span>Distance rays</span>
        <span class="brain-legend__item"><span class="brain-legend__swatch brain-legend__swatch--imu"></span>IMU init heading</span>
      </div>
    </section>

    <section class="brain-panel brain-panel--telemetry">
      <div class="brain-panel__title">
        <span>POSE AND TELEMETRY</span>
        <span class="brain-status-pill">LIVE</span>
      </div>

      <div class="brain-card-grid">
        <div class="brain-card">
          <div class="brain-card__label">COMBINED</div>
          <strong>X +18.4 in</strong>
          <strong>Y -11.2 in</strong>
          <span>H 312.4 deg</span>
        </div>
        <div class="brain-card">
          <div class="brain-card__label">PURE MCL</div>
          <strong>X +17.9 in</strong>
          <strong>Y -10.8 in</strong>
          <span>Particle-filter estimate</span>
        </div>
      </div>

      <div class="brain-tile-grid">
        <div class="brain-tile">
          <span>LEFT</span>
          <strong>24.6 in</strong>
          <em>c71</em>
        </div>
        <div class="brain-tile">
          <span>RIGHT</span>
          <strong>10.4 in</strong>
          <em>c66</em>
        </div>
        <div class="brain-tile">
          <span>FRONT</span>
          <strong>31.8 in</strong>
          <em>c89</em>
        </div>
        <div class="brain-tile">
          <span>BACK</span>
          <strong>13.1 in</strong>
          <em>c58</em>
        </div>
      </div>

      <div class="brain-note-strip">
        <div><strong>View behavior:</strong> sightlines and measured rays appear on <code>PureMcl</code> and <code>Combined</code>.</div>
        <div><strong>Geometry source:</strong> robot box uses <code>ROBOT_LENGTH</code> and <code>ROBOT_WIDTH</code>.</div>
        <div><strong>Distance model:</strong> rotated obstacles and height-gated ray hits come from the MCL obstacle set.</div>
      </div>
    </section>
  </div>
</div>

## What This Mockup Represents

- The left panel is the minimap direction for the `ODOM` runtime page.
- The grey obstacles are the currently modeled distance-obstacle surfaces, not every field element.
- The center `X` goal is represented as only two intersecting bars.
- The loaders are shown as one small edge square in each quadrant at `y = +/-48 in`.
- The robot is drawn as a box using the actual config dimensions, not a point marker.
- Sensor rays are shown the way the live `PureMcl` and `Combined` views intend to communicate them.

## Source of Truth

- UI render path: [`src/ui/display.cpp`](../src/ui/display.cpp)
- Field obstacle geometry: [`include/config.h`](../include/config.h)
- MCL distance raycast: [`include/localization/distance.h`](../include/localization/distance.h)
