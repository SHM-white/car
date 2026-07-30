# Ground Station ESP32-S3 Design System

## 1. Atmosphere & Identity

A compact 800x480 operator cockpit for an embedded air-ground station. The interface prioritizes fast task selection, link freshness, and mission-state awareness over visual decoration. The signature is a calm light field panel paired with a dense dark header strip: high-contrast status information stays glanceable even when the operator is focused on the physical vehicle, not the screen.

## 2. Color

| Role | Token | Value | Usage |
|------|-------|-------|-------|
| Surface/background | --surface-background | #F3F5F7 | Main canvas |
| Surface/panel | --surface-panel | #FFFFFF | Left task panel, track view box |
| Surface/elevated | --surface-elevated | #E7EFF5 | Mission state band |
| Surface/top-bar | --surface-top-bar | #1D252C | Top header bar |
| Text/primary | --text-primary | #182026 | Primary labels and values |
| Text/secondary | --text-secondary | #5F6B73 | Section headings and muted labels |
| Text/top-bar | --text-top-bar | #FFFFFF | Top header title text |
| Text/top-bar-subtle | --text-top-bar-subtle | #D9E0E5 | Top header link freshness text |
| Border/primary | --border-primary | #D5DBE0 | Vertical divider |
| Status/primary | --status-primary | #1769AA | Primary task action |
| Status/primary-pressed | --status-primary-pressed | #0F4F82 | Pressed primary action |
| Status/checked | --status-checked | #15803D | Selected or checked action state |
| Status/disabled | --status-disabled | #A8B0B6 | Disabled action |
| Status/ok | --status-ok | #15803D | Nominal status |
| Status/warn | --status-warn | #A75B00 | Stale, pending, or degraded status |
| Status/fault | --status-fault | #B42318 | No-data or fault status |

Rules:
- Reserve green for nominal system state and checked confirmation.
- Reserve amber for pending selection, degraded state, or stale-but-known condition.
- Reserve red only for missing data, lost link, or active fault.
- Blue is used only for primary interactive action and ROS phase emphasis.

## 3. Typography

| Level | Size | Weight | Usage |
|-------|------|--------|-------|
| Header/title | 26 | Montserrat | Station title, local state value, selected task value |
| Section/label | 16 | Montserrat | Section heading, button label, status pair label, telemetry label |
| Micro | 12 | Montserrat | FPS readout, track-view section label |

Rules:
- ASCII-only visible labels to avoid pulling a CJK glyph set into the mission firmware.
- Monospace is not required for this control set because values are short and fixed-field padding is already used.
- Keep one font family (`lv_font_montserrat_*`) across the ground station UI.

## 4. Spacing & Layout

Base unit: 4px, but the active spacing system in code is a fixed-pixel layout tuned for 800x480.

| Token | Value | Usage |
|-------|-------|-------|
| --space-compact | 4px | Inner label offset inside panels |
| --space-card | 10-14px | Left-panel section heading inset |
| --space-row | 24-28px | Status pair / telemetry row stride |
| --space-action | 48px | Task button height |
| --space-gutter | 1px | Vertical divider between left and main panels |
| --space-fault | 50px | Bottom fault/FPS bar height |

Layout zones:
- Top header: y0-56, full width.
- Left task panel: x0-255, y56-430.
- Mission state band: x255-800, y56-128.
- System status column: x279-500, y146-320.
- Track view panel: x558-762, y134-274.
- Right telemetry column: x535-775, y280-424.
- Fault/FPS bar: y430-480, full width.

Rules:
- Maintain generous right-side padding so telemetry and status labels never collide with the track view.
- Button rows must remain comfortably touchable; minimum action height is 48px.
- The divider is intentionally only 1px because the panel background shift already separates hierarchy.

## 5. Components

### Task Button
- Structure: rounded rect action with centered 16px Montserrat label.
- Variants: TASK 1 / TASK 2 / TEST.
- Spacing: 24px left inset, 207px width, 48px height.
- States: default, pressed, checked, disabled.
- Accessibility: high-contrast label over action surface, touch area >= 48px height.
- Motion: none; embedded surface should stay instantaneous.

### Status Pair Row
- Structure: left label + right value label on the same baseline.
- Variants: CAR LINK, DRONE LINK, VISION, ROS READY.
- Spacing: left label at x279, value label at x402, width 110 each.
- States: green nominal, amber degraded, red missing/stale.
- Accessibility: color plus explicit text state so status is readable without color discrimination alone.

### Track View Panel
- Structure: white rounded container with border, title label, and large circular position marker.
- Variants: single simplified route representation using a rounded rectangle outline.
- Spacing: title inset 14px, marker 14x14.
- States: muted marker when stale/no data, blue marker when fresh.
- Accessibility: symbolic overview only; text telemetry remains the source of truth.

### Selected/Pending Task Area
- Structure: section label plus large selected value plus smaller pending value.
- Variants: ROS model state, local test overlay.
- Spacing: stacked vertically in left panel with 24px horizontal inset.
- States: default numeric state, local test override label.
- Accessibility: test mode is shown explicitly so the operator cannot confuse it with ROS-confirmed selection.

## 6. Motion & Interaction

Timing:
| Type | Duration | Usage |
|------|----------|-------|
| State update | 100ms render cycle | Task state, telemetry, link freshness, FPS sample at 1s |

Rules:
- No animation layer is needed for this cockpit UI.
- Feedback comes from immediate label and color state changes, not from decorative motion.
- Pressed and checked states exist on interactive buttons, but screen transitions do not.

## 7. Depth & Surface

Strategy: borders plus tonal panel shifts.

- The header bar uses a dark background for strong separation.
- The mission band uses a tinted light-gray panel.
- Task area and track view use white surfaces with subtle border separation.
- The fault bar switches between green and red background as a full-width status signal.

Rules:
- Do not add shadows or glass effects to an embedded control surface.
- Use border width only where white meets white (track view and vertical divider).
- Use background tone changes to separate sections rather than elevation metaphors.

## 8. Accessibility Constraints & Accepted Debt

Constraints:
- Contrast floor: interactive buttons and critical labels must remain legible on their background.
- Status meaning must be communicated by both color and text.
- Touch targets should remain at least 48px high for the primary task actions.

Accepted Debt:
| Item | Location | Why accepted | Owner / Exit |
|------|----------|--------------|--------------|
| No audio/haptic feedback | Ground station UI | Embedded hardware constraints and current firmware scope | Add only if UX testing on the real device proves necessary |
| Symbolic track view only | Track view panel | Backend does not yet expose pose/absolute course truth | Replace with calibrated map once a real track/position model exists |
