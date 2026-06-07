(() => {
    "use strict";

    const WS_URL        = `ws://${location.hostname || "localhost"}:9090`;
    const WINDOW_SECONDS = 10;
    const MAX_SAMPLES    = 5000;
    const REDRAW_HZ      = 30;

    const COLOR_EST    = "#0A0A0A";
    const COLOR_ACT    = "#FF6C00";
    const COLOR_TARGET = "#D97706";   // amber — reference / target line
    const COLOR_GRID   = "#E5E2DD";
    const LINE_WIDTH   = 2.0;

    // ── App state ────────────────────────────────────────────────────────────
    const state = {
        paused:          false,
        profileActive:   false,
        profileStartTs:  0,
        latestActual:    null,    // latest actual_states msg data
        timeRef:         null,    // epoch-second reference; null = use first stamp
        cropMode:        false,
        rightTab:        "profile",
        trackEvents:     true,
        targetRef:       null,    // current target_rad for profile position reference line
        livePosUnit:     "rad",   // left panel unit: "rad" | "deg"
        profilePosUnit:  "rad",   // right panel unit: "rad" | "deg"
    };

    // ── Data buffers ─────────────────────────────────────────────────────────
    const live = {
        raw_time: [],
        est_pos: [], est_vel: [], est_acc: [],
        act_pos: [], act_vel: [], act_acc: [],   // nearest-neighbour to est time
    };

    const profile = {
        time:    [],   // seconds since profileStartTs
        est_pos: [], est_vel: [], est_acc: [],
        act_pos: [], act_vel: [], act_acc: [],
    };

    // Snapshot filled on crop completion
    const zoom = {
        time:    [],
        est_pos: [], est_vel: [], est_acc: [],
        act_pos: [], act_vel: [], act_acc: [],
    };

    // ── Helpers ───────────────────────────────────────────────────────────────
    function displayRef() {
        return state.timeRef ?? (live.raw_time[0] ?? 0);
    }

    function posScale(which) {
        const unit = which === "live" ? state.livePosUnit : state.profilePosUnit;
        return unit === "deg" ? (180 / Math.PI) : 1.0;
    }

    function applyPos(arr, off, scale) {
        return arr.map(v => (v != null ? (v - off) * scale : null));
    }

    const TWO_PI = 2 * Math.PI;
    function wrapAngle(v) {
        // null-safe; ((v % 2π)+2π)%2π normalizes any real (incl. negatives) into [0, 2π)
        return v == null ? null : ((v % TWO_PI) + TWO_PI) % TWO_PI;
    }

    function updatePosLabels() {
        const lUnit = state.livePosUnit    === "deg" ? "[deg]" : "[rad]";
        const pUnit = state.profilePosUnit === "deg" ? "[deg]" : "[rad]";
        document.querySelectorAll(".pos-unit-label-live").forEach(el => el.textContent = lUnit);
        document.querySelectorAll(".pos-unit-label-profile").forEach(el => el.textContent = pUnit);
    }

    // ── CSV export ───────────────────────────────────────────────────────────
    const btnSaveCsv = document.getElementById("btn-save-csv");

    function updateSaveCsvBtn() {
        const tab = state.rightTab;
        const hasData = (tab === "profile" && profile.time.length > 0) ||
                        (tab === "zoom"    && zoom.time.length   > 0);
        btnSaveCsv.disabled = !hasData;
    }

    function buildCSV(source) {
        const buf    = source === "profile" ? profile : zoom;
        const pScale = posScale("profile");
        const unit   = state.profilePosUnit;
        const header = `time_s,est_pos_${unit},act_pos_${unit},est_vel_rad_s,act_vel_rad_s,est_acc_rad_s2,act_acc_rad_s2`;
        const rows = buf.time.map((t, i) => [
            t.toFixed(4),
            buf.est_pos[i] != null ? (buf.est_pos[i] * pScale).toFixed(6) : '',
            buf.act_pos[i] != null ? (buf.act_pos[i] * pScale).toFixed(6) : '',
            buf.est_vel[i] != null ?  buf.est_vel[i].toFixed(6)           : '',
            buf.act_vel[i] != null ?  buf.act_vel[i].toFixed(6)           : '',
            buf.est_acc[i] != null ?  buf.est_acc[i].toFixed(6)           : '',
            buf.act_acc[i] != null ?  buf.act_acc[i].toFixed(6)           : '',
        ].join(','));
        return [header, ...rows].join('\n');
    }

    async function saveCSV(filename, content) {
        const blob = new Blob([content], { type: 'text/csv' });
        if (window.showSaveFilePicker) {
            try {
                const handle = await window.showSaveFilePicker({
                    suggestedName: filename,
                    types: [{ description: 'CSV file', accept: { 'text/csv': ['.csv'] } }],
                });
                const writable = await handle.createWritable();
                await writable.write(blob);
                await writable.close();
                return;
            } catch (e) {
                if (e.name === 'AbortError') return;
            }
        }
        const a = document.createElement('a');
        a.href = URL.createObjectURL(blob);
        a.download = filename;
        a.click();
        URL.revokeObjectURL(a.href);
    }

    btnSaveCsv.addEventListener("click", () => {
        const tab = state.rightTab;
        if (tab !== "profile" && tab !== "zoom") return;
        const ts       = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
        const filename = `${tab}_${ts}.csv`;
        saveCSV(filename, buildCSV(tab));
    });

    updateSaveCsvBtn();

    // ── Tooltip element ───────────────────────────────────────────────────────
    const tooltip = document.getElementById("plot-tooltip");

    function updateTooltip(u) {
        const idx = u.cursor.idx;
        if (idx == null) { tooltip.classList.add("hidden"); return; }

        // u.data[0] is already the display time (raw_time - ref from redraw()),
        // so use it directly without re-applying any reference.
        const t   = u.data[0][idx] != null ? u.data[0][idx].toFixed(2) : "—";
        const est = u.data[1]?.[idx] != null ? u.data[1][idx].toFixed(4) : "—";

        let text = `t: ${t}s  est: ${est}`;
        // Right-panel plots have no "actual" series (idx 2 may be the target line);
        // only show the act segment when series idx 2 is actually the actual series.
        const s2 = u.series[2];
        if (s2 && s2.label === "actual") {
            const act = u.data[2]?.[idx] != null ? u.data[2][idx].toFixed(4) : "—";
            text += `  act: ${act}`;
        }
        tooltip.textContent = text;
        tooltip.style.left  = (u.rect.left + u.cursor.left + 14) + "px";
        tooltip.style.top   = (u.rect.top  + u.cursor.top  - 32) + "px";
        tooltip.classList.remove("hidden");
    }

    // Called by cursor.move — snaps the horizontal crosshair to the estimated series y-value
    function snapCursorToEst(u, mouseLeft, mouseTop) {
        const idx = u.cursor.idx;
        if (idx != null && u.data[1]?.[idx] != null)
            return [mouseLeft, u.valToPos(u.data[1][idx], "y")];
        return [mouseLeft, mouseTop];
    }

    // ── uPlot options factories ───────────────────────────────────────────────
    function axisOpts() {
        return [
            {
                stroke: COLOR_EST,
                grid:   { stroke: COLOR_GRID, width: 0.5 },
                ticks:  { stroke: COLOR_EST,  width: 1 },
                label:  "Time [s]",
                size:   50,
            },
            {
                stroke: COLOR_EST,
                grid:   { stroke: COLOR_GRID, width: 0.5 },
                ticks:  { stroke: COLOR_EST,  width: 1 },
                size:   52,
            },
        ];
    }

    function makeDualOpts(el, allowDragX, allowDragY) {
        const rect = el.getBoundingClientRect();
        return {
            width:   rect.width  || 400,
            height:  rect.height || 160,
            padding: [0, 0, 0, 0],
            scales:  { x: { time: false } },
            series: [
                {},
                { stroke: COLOR_EST, width: LINE_WIDTH, label: "estimated" },
                { stroke: COLOR_ACT, width: LINE_WIDTH, label: "actual"    },
            ],
            axes:   axisOpts(),
            cursor: {
                move:   snapCursorToEst,
                drag:   { x: allowDragX ?? false, y: allowDragY ?? false },
                points: { size: 6, width: 1 },
            },
            legend: { show: false },
            hooks:  { setCursor: [updateTooltip] },
        };
    }

    // Right-panel factory: same as makeDualOpts but without the "actual" series.
    function makeEstOpts(el, allowDragX, allowDragY) {
        const opts = makeDualOpts(el, allowDragX, allowDragY);
        opts.series.splice(2, 1);   // drop "actual"; leaves [x, estimated]
        return opts;
    }

    function makeProfilePosOpts(el) {
        const opts = makeEstOpts(el, true, false);
        opts.series.push({
            stroke: COLOR_TARGET,
            width:  1.5,
            dash:   [6, 4],
            label:  "target",
        });
        return opts;
    }

    // ── Plot instances ────────────────────────────────────────────────────────
    const elPos  = document.getElementById("plot-position");
    const elVel  = document.getElementById("plot-velocity");
    const elAcc  = document.getElementById("plot-acceleration");
    const elPPos = document.getElementById("plot-profile-position");
    const elPVel = document.getElementById("plot-profile-velocity");
    const elPAcc = document.getElementById("plot-profile-acceleration");
    const elZPos = document.getElementById("plot-zoom-position");
    const elZVel = document.getElementById("plot-zoom-velocity");
    const elZAcc = document.getElementById("plot-zoom-acceleration");

    const plots = {
        position:     new uPlot(makeDualOpts(elPos),  [[], [], []], elPos),
        velocity:     new uPlot(makeDualOpts(elVel),  [[], [], []], elVel),
        acceleration: new uPlot(makeDualOpts(elAcc),  [[], [], []], elAcc),
        profilePos:   new uPlot(makeProfilePosOpts(elPPos), [[], [], []], elPPos),
        profileVel:   new uPlot(makeEstOpts(elPVel, true, false), [[], []], elPVel),
        profileAcc:   new uPlot(makeEstOpts(elPAcc, true, false), [[], []], elPAcc),
        zoomPos:      new uPlot(makeEstOpts(elZPos, true, false), [[], []], elZPos),
        zoomVel:      new uPlot(makeEstOpts(elZVel, true, false), [[], []], elZVel),
        zoomAcc:      new uPlot(makeEstOpts(elZAcc, true, false), [[], []], elZAcc),
    };

    // ── Canvas aria-labels (accessibility) ───────────────────────────────────
    const canvasLabels = {
        position:     "Live position plot",
        velocity:     "Live velocity plot",
        acceleration: "Live acceleration plot",
        profilePos:   "Profile position plot",
        profileVel:   "Profile velocity plot",
        profileAcc:   "Profile acceleration plot",
        zoomPos:      "Zoom position plot",
        zoomVel:      "Zoom velocity plot",
        zoomAcc:      "Zoom acceleration plot",
    };
    for (const [key, label] of Object.entries(canvasLabels))
        plots[key].root.querySelectorAll("canvas").forEach(c => c.setAttribute("aria-label", label));

    // ── Clickable legend toggles ──────────────────────────────────────────────
    for (const { el, plot } of [
        { el: elPos,  plot: plots.position     },
        { el: elVel,  plot: plots.velocity     },
        { el: elAcc,  plot: plots.acceleration },
        { el: elPPos, plot: plots.profilePos   },
        { el: elPVel, plot: plots.profileVel   },
        { el: elPAcc, plot: plots.profileAcc   },
        { el: elZPos, plot: plots.zoomPos      },
        { el: elZVel, plot: plots.zoomVel      },
        { el: elZAcc, plot: plots.zoomAcc      },
    ]) {
        const container = el.closest(".plot-container");
        if (!container) continue;
        container.querySelectorAll(".legend-est, .legend-actual").forEach(span => {
            const idx = span.classList.contains("legend-est") ? 1 : 2;
            let visible = true;
            span.setAttribute("role", "button");
            span.setAttribute("tabindex", "0");
            span.addEventListener("click", () => {
                visible = !visible;
                plot.setSeries(idx, { show: visible });
                span.style.opacity = visible ? "" : "0.3";
            });
            span.addEventListener("keydown", e => {
                if (e.key === "Enter" || e.key === " ") { e.preventDefault(); span.click(); }
            });
        });
    }

    // ── Plot resize ───────────────────────────────────────────────────────────
    // One function sizes every plot using clientWidth/clientHeight (content area,
    // border excluded). Called on window resize and once after first paint.
    function resizePlots() {
        const allPlots = [
            [plots.position,     elPos],
            [plots.velocity,     elVel],
            [plots.acceleration, elAcc],
            [plots.profilePos,   elPPos],
            [plots.profileVel,   elPVel],
            [plots.profileAcc,   elPAcc],
            [plots.zoomPos,      elZPos],
            [plots.zoomVel,      elZVel],
            [plots.zoomAcc,      elZAcc],
        ];
        for (const [p, el] of allPlots) {
            const w = el.clientWidth;
            const h = el.clientHeight;
            if (w > 10 && h > 10) p.setSize({ width: w, height: h });
        }
    }
    window.addEventListener("resize", resizePlots);
    new ResizeObserver(() => requestAnimationFrame(resizePlots))
        .observe(document.querySelector(".panel-right"));

    // Track which right-panel plots the user has zoomed into.
    // redraw() passes resetScales=false for locked plots so the zoom survives the 30 Hz tick.
    // Double-click unlocks and resets to auto-fit.
    const zoomedPlots = new Set();

    for (const [plot, el] of [
        [plots.profilePos, elPPos], [plots.profileVel, elPVel], [plots.profileAcc, elPAcc],
        [plots.zoomPos, elZPos], [plots.zoomVel, elZVel], [plots.zoomAcc, elZAcc],
    ]) {
        let dragStartX = null;
        el.addEventListener("mousedown", e => { dragStartX = e.clientX; });
        el.addEventListener("mouseup",   e => {
            if (dragStartX !== null && Math.abs(e.clientX - dragStartX) > 5)
                zoomedPlots.add(plot);
            dragStartX = null;
        });
        el.addEventListener("dblclick", () => {
            zoomedPlots.delete(plot);
            plot.setScale("x", { min: null, max: null });
            plot.setScale("y", { min: null, max: null });
        });
    }

    // ── Data ingestion ────────────────────────────────────────────────────────
    function pushLive(msg) {
        const { stamp, position, velocity, acceleration } = msg.data;

        if (state.timeRef === null && live.raw_time.length === 0)
            state.timeRef = stamp;

        live.raw_time.push(stamp);
        live.est_pos.push(position);
        live.est_vel.push(velocity);
        live.est_acc.push(acceleration);

        const a = state.latestActual;
        live.act_pos.push(a ? a.actual_position     : null);
        live.act_vel.push(a ? a.actual_velocity     : null);
        live.act_acc.push(a ? a.actual_acceleration : null);

        // Rolling window trim
        const cutoff = stamp - WINDOW_SECONDS;
        while (live.raw_time.length && live.raw_time[0] < cutoff)
            for (const arr of Object.values(live)) arr.shift();
        while (live.raw_time.length > MAX_SAMPLES)
            for (const arr of Object.values(live)) arr.shift();

        // Profile collection (relative time from trigger START)
        if (state.profileActive) {
            profile.time.push(stamp - state.profileStartTs);
            profile.est_pos.push(position);
            profile.est_vel.push(velocity);
            profile.est_acc.push(acceleration);
            profile.act_pos.push(a ? a.actual_position  : null);
            profile.act_vel.push(a ? a.actual_velocity  : null);
            profile.act_acc.push(a ? a.actual_acceleration : null);
        }
    }

    function onActualStates(msg) {
        state.latestActual = msg.data;
    }

    function startProfile({ label = "", profile_id = "", expected_duration = 0, stamp = null } = {}) {
        const ts = stamp ?? live.raw_time[live.raw_time.length - 1] ?? (Date.now() / 1000);
        state.profileActive  = true;
        state.profileStartTs = ts;
        zoomedPlots.delete(plots.profilePos);
        zoomedPlots.delete(plots.profileVel);
        zoomedPlots.delete(plots.profileAcc);
        for (const arr of Object.values(profile)) arr.length = 0;

        document.getElementById("profile-idle").classList.add("hidden");
        document.getElementById("profile-active").classList.remove("hidden");
        document.getElementById("profile-label").textContent    = label        || "(unnamed)";
        document.getElementById("profile-id").textContent       = profile_id   || "—";
        document.getElementById("profile-duration").textContent =
            expected_duration > 0 ? expected_duration.toFixed(1) : "—";

        if (state.rightTab !== "profile") switchTab("profile");
        requestAnimationFrame(resizePlots);
    }

    function stopProfile() {
        state.profileActive = false;
        updateSaveCsvBtn();
    }

    const EXPERIMENT_LABELS = {
        point_to_point: "Point to Point",
        pick_place:     "Pick & Place",
        performance:    "Performance",
        precision:      "Precision",
    };

    function clearEval() {
        state.targetRef = null;
        document.getElementById("eval-live").classList.add("hidden");
        document.getElementById("eval-summary").classList.add("hidden");
        document.getElementById("eval-metrics-rows").innerHTML = "";
        document.getElementById("eval-summary-rows").innerHTML = "";
    }

    function startExperiment(data) {
        const label = EXPERIMENT_LABELS[data.action] ?? data.action;
        startProfile({ label, stamp: data.stamp });
        clearEval();
    }

    function stopExperiment() { stopProfile(); }

    function onEventTrigger(msg) {
        if (!state.trackEvents) return;
        const data = msg.data;
        if (!data.action) return;
        if (data.action === "stop") stopExperiment();
        else startExperiment(data);
    }

    // ── Redraw loop (30 Hz, decoupled from message rate) ─────────────────────
    function redraw() {
        if (state.paused) return;

        const ref  = displayRef();
        const tData = live.raw_time.map(t => t - ref);
        const pEst = 0, pAct = 0;
        const lScale = posScale("live");
        const pScale = posScale("profile");

        plots.position    .setData([tData,
            applyPos(live.est_pos.map(wrapAngle), pEst, lScale),
            applyPos(live.act_pos, pAct, lScale)]);
        plots.velocity    .setData([tData, live.est_vel, live.act_vel]);
        plots.acceleration.setData([tData, live.est_acc, live.act_acc]);

        if (state.profileActive || profile.time.length > 0) {
            const targetData = profile.time.map(() =>
                state.targetRef != null ? state.targetRef * pScale : null);
            plots.profilePos.setData([profile.time,
                applyPos(profile.est_pos, pEst, pScale),
                targetData], !zoomedPlots.has(plots.profilePos));
            plots.profileVel.setData([profile.time, profile.est_vel],
                !zoomedPlots.has(plots.profileVel));
            plots.profileAcc.setData([profile.time, profile.est_acc],
                !zoomedPlots.has(plots.profileAcc));
        }

        if (zoom.time.length > 0) {
            plots.zoomPos.setData([zoom.time, applyPos(zoom.est_pos, 0, pScale)],
                !zoomedPlots.has(plots.zoomPos));
            plots.zoomVel.setData([zoom.time, zoom.est_vel],
                !zoomedPlots.has(plots.zoomVel));
            plots.zoomAcc.setData([zoom.time, zoom.est_acc],
                !zoomedPlots.has(plots.zoomAcc));
        }

        if (state.profileActive && live.raw_time.length) {
            document.getElementById("profile-elapsed").textContent =
                (live.raw_time[live.raw_time.length - 1] - state.profileStartTs).toFixed(1);
        }
    }
    setInterval(redraw, 1000 / REDRAW_HZ);

    // ── Eval helpers ─────────────────────────────────────────────────────────

    const METRIC_LABEL_MAP = {
        // shared
        target_rad:              "Target [rad]",
        elapsed_s:               "Elapsed [s]",
        // position / error
        current_pos_rad:         "Current pos [rad]",
        current_error_rad:       "Error [rad]",
        final_error_rad:         "Final error [rad]",
        avg_error_rad:           "Avg error [rad]",
        mean_error_rad:          "Mean error [rad]",
        std_error_rad:           "Std error [rad]",
        max_error_rad:           "Max error [rad]",
        // motion
        overshoot_pct:           "Overshoot [%]",
        settling_time_s:         "Settling time [s]",
        // speed / accel (unsigned)
        peak_speed_rad_s:        "Peak speed [rad/s]",
        peak_accel_rad_s2:       "Peak accel [rad/s²]",
        current_speed_rad_s:     "Curr speed [rad/s]",
        commanded_speed_rad_s:   "Cmd speed [rad/s]",
        commanded_accel_rad_s2:  "Cmd accel [rad/s²]",
        // counters
        current_waypoint:        "Waypoint",
        total_waypoints:         "Total WPs",
        passed:                  "Passed WPs",
        failed:                  "Failed WPs",
        trials_done:             "Trials done",
        trials_total:            "Total trials",
        num_trials:              "Trials",
    };

    const INT_FIELDS = new Set([
        "current_waypoint", "total_waypoints", "trials_done", "trials_total",
        "passed", "failed", "num_trials", "waypoint",
    ]);

    function fmtVal(k, v) {
        if (typeof v === "boolean") return v ? "true" : "false";
        if (INT_FIELDS.has(k))     return String(Math.round(v));
        if (typeof v === "number") return v.toFixed(4);
        return String(v);
    }

    function appendRow(container, k, v) {
        const isPass = k.startsWith("pass_");
        const label  = METRIC_LABEL_MAP[k] ??
                       (isPass ? k.slice(5).replace(/_/g, " ") : k.replace(/_/g, " "));
        // Position fields already arrive in the zeroed frame from the evaluator.
        const displayVal = v;
        const row    = document.createElement("div");
        row.className = "eval-row" + (isPass ? (v ? " pass-ok" : " pass-fail") : "");
        row.innerHTML = `<span class="eval-key">${label}</span>` +
                        `<span class="eval-val">${isPass ? (v ? "PASS" : "FAIL") : fmtVal(k, displayVal)}</span>`;
        container.appendChild(row);
    }

    function renderMetrics(d) {
        const rows = document.getElementById("eval-metrics-rows");
        rows.innerHTML = "";
        const SKIP = new Set(["stamp", "action"]);
        for (const [k, v] of Object.entries(d))
            if (!SKIP.has(k) && v != null) appendRow(rows, k, v);
    }

    function onEvalLive(msg) {
        const d = msg.data;
        if (d.target_rad != null) state.targetRef = d.target_rad;
        document.getElementById("eval-summary").classList.add("hidden");
        document.getElementById("eval-live-label").textContent =
            EXPERIMENT_LABELS[d.action] ?? d.action ?? "Experiment";
        renderMetrics(d);
        document.getElementById("eval-live").classList.remove("hidden");
    }

    function renderSummary(d) {
        const rows = document.getElementById("eval-summary-rows");
        rows.innerHTML = "";
        const SKIP = new Set(["stamp", "action", "details"]);
        for (const [k, v] of Object.entries(d))
            if (!SKIP.has(k) && v != null) appendRow(rows, k, v);

        if (Array.isArray(d.details) && d.details.length > 0) {
            const titleEl = document.createElement("div");
            titleEl.className = "eval-waypoint-title";
            titleEl.textContent = "Waypoint Details";
            rows.appendChild(titleEl);
            for (const wp of d.details) {
                const groupEl = document.createElement("div");
                groupEl.className = "eval-waypoint-group";
                const allPass = wp.pass_error && wp.pass_overshoot && wp.pass_settling;
                const hdr = document.createElement("div");
                hdr.className = "eval-row" + (allPass ? " pass-ok" : " pass-fail");
                hdr.innerHTML = `<span class="eval-key">Waypoint ${wp.waypoint}</span>` +
                                `<span class="eval-val">${allPass ? "PASS" : "FAIL"}</span>`;
                groupEl.appendChild(hdr);
                const SKIP_WP = new Set(["waypoint"]);
                for (const [k, v] of Object.entries(wp))
                    if (!SKIP_WP.has(k) && v != null) appendRow(groupEl, k, v);
                rows.appendChild(groupEl);
            }
        }
    }

    function onEvalSummary(msg) {
        const d = msg.data;
        stopProfile();
        document.getElementById("eval-live").classList.add("hidden");
        document.getElementById("eval-summary-action").textContent =
            EXPERIMENT_LABELS[d.action] ?? d.action ?? "Experiment";
        renderSummary(d);
        document.getElementById("eval-summary").classList.remove("hidden");
        switchTab("profile");
    }

    // ── Criteria tab ─────────────────────────────────────────────────────────
    const CRITERIA_LABEL_MAP = {
        min_speed:           "Min speed [rad/s]",
        min_acceleration:    "Min accel [rad/s²]",
        max_avg_error_rad:   "Max avg error [rad]",
        max_overshoot_pct:   "Max overshoot [%]",
        max_settling_time_s: "Max settling [s]",
        settling_band_pct:   "Settling band [%]",
    };

    let lastEditedInput = null;

    function onCriteriaSnapshot(msg) {
        const container = document.getElementById("criteria-rows");
        container.innerHTML = "";
        for (const [key, val] of Object.entries(msg.data)) {
            const label = CRITERIA_LABEL_MAP[key] ?? key.replace(/_/g, " ");
            const row = document.createElement("div");
            row.className = "criteria-row";
            row.innerHTML =
                `<span class="criteria-key">${label}</span>` +
                `<input class="criteria-input" type="number" step="any"` +
                ` value="${val}" data-key="${key}" data-original="${val}">`;
            container.appendChild(row);
        }
        container.querySelectorAll(".criteria-input").forEach(input => {
            input.addEventListener("keydown", e => {
                if (e.key === "Enter") input.blur();
                if (e.key === "Escape") { input.value = input.dataset.original; input.blur(); }
            });
            input.addEventListener("blur", () => {
                const val = parseFloat(input.value);
                if (isNaN(val) || val < 0) { input.value = input.dataset.original; return; }
                if (val === parseFloat(input.dataset.original)) return;
                if (ws && ws.readyState === WebSocket.OPEN) {
                    lastEditedInput = input;
                    ws.send(JSON.stringify({ command: "criteria_update", data: { [input.dataset.key]: val } }));
                }
            });
        });
        document.getElementById("criteria-idle").classList.add("hidden");
        document.getElementById("criteria-active").classList.remove("hidden");
    }

    function onCriteriaAck(msg) {
        const d = msg.data;
        if (lastEditedInput) {
            if (d.success) {
                lastEditedInput.dataset.original = lastEditedInput.value;
                flashCriteriaInput(lastEditedInput, true);
            } else {
                lastEditedInput.value = lastEditedInput.dataset.original;
                flashCriteriaInput(lastEditedInput, false);
            }
            lastEditedInput = null;
        }
        if (d.success && d.criteria) {
            for (const [key, val] of Object.entries(d.criteria)) {
                const input = document.querySelector(`.criteria-input[data-key="${key}"]`);
                if (input && input !== document.activeElement) {
                    input.value = val;
                    input.dataset.original = val;
                }
            }
        }
    }

    function flashCriteriaInput(input, ok) {
        const cls = ok ? "flash-ok" : "flash-err";
        input.classList.remove("flash-ok", "flash-err");
        void input.offsetWidth;
        input.classList.add(cls);
        setTimeout(() => input.classList.remove(cls), 600);
    }

    // ── WebSocket ─────────────────────────────────────────────────────────────
    const statusDot  = document.getElementById("status-dot");
    const statusText = document.getElementById("status-text");
    document.getElementById("ws-url").textContent = WS_URL;

    let ws = null;

    function connect() {
        ws = new WebSocket(WS_URL);

        ws.addEventListener("open", () => {
            statusDot.className    = "dot connected";
            statusText.textContent = "Connected";
        });

        ws.addEventListener("close", () => {
            statusDot.className    = "dot disconnected";
            statusText.textContent = "Disconnected — retrying";
            ws = null;
            setTimeout(connect, 1000);
        });

        ws.addEventListener("error", () => { /* close handler retries */ });

        ws.addEventListener("message", ev => {
            let msg;
            try { msg = JSON.parse(ev.data); } catch (_) { return; }

            switch (msg.topic) {
                case "estimated_states": pushLive(msg);        break;
                case "actual_states":   onActualStates(msg);  break;
                case "event_trigger":   onEventTrigger(msg);  break;
                case "eval_live":          onEvalLive(msg);          break;
                case "eval_summary":       onEvalSummary(msg);       break;
                case "criteria_snapshot":  onCriteriaSnapshot(msg);  break;
                case "criteria_ack":       onCriteriaAck(msg);       break;
                case "time_sync":
                    state.timeRef = msg.data.ref_stamp;
                    break;
            }
        });
    }
    connect();

    // ── Tab switching ─────────────────────────────────────────────────────────
    function switchTab(name) {
        state.rightTab = name;
        document.querySelectorAll(".tab-btn").forEach(btn => {
            btn.classList.toggle("active", btn.dataset.tab === name);
        });
        document.querySelectorAll(".tab-content").forEach(tc => {
            tc.classList.toggle("hidden", tc.id !== `tab-${name}`);
        });
        updateSaveCsvBtn();
        requestAnimationFrame(resizePlots);
    }

    document.querySelectorAll(".tab-btn").forEach(btn => {
        btn.addEventListener("click", () => switchTab(btn.dataset.tab));
    });

    // ── Crop / zoom feature ───────────────────────────────────────────────────
    const cropState = {
        dragging:     false,
        startRaw:     0,
        startOverlayX: 0,
        activePlot:   null,
    };

    // One selection-rect div per left-panel overlay (kept in sync)
    const overlayIds  = ["co-position", "co-velocity", "co-acceleration"];
    const overlayEls  = overlayIds.map(id => document.getElementById(id));
    const selRects    = overlayEls.map(() => {
        const d = document.createElement("div");
        d.className = "crop-selection hidden";
        return d;
    });
    overlayEls.forEach((ov, i) => ov.appendChild(selRects[i]));

    // Map each overlay to the corresponding left plot
    const leftPlots = [plots.position, plots.velocity, plots.acceleration];

    function enterCropMode() {
        state.cropMode = true;
        overlayEls.forEach(ov => ov.classList.add("active"));
        document.getElementById("btn-crop").classList.add("crop-active");
        document.getElementById("btn-crop").textContent = "Exit Crop";
    }

    function exitCropMode() {
        state.cropMode = false;
        cropState.dragging = false;
        overlayEls.forEach(ov => ov.classList.remove("active"));
        selRects.forEach(r => { r.classList.add("hidden"); r.style.cssText = ""; });
        document.getElementById("btn-crop").classList.remove("crop-active");
        document.getElementById("btn-crop").textContent = "Crop";
    }

    function updateSelectionRects(startPx, endPx) {
        const left  = Math.min(startPx, endPx);
        const width = Math.abs(endPx - startPx);
        selRects.forEach(r => {
            r.classList.remove("hidden");
            r.style.left  = left + "px";
            r.style.width = width + "px";
        });
    }

    function applyZoom(t0Raw, t1Raw) {
        const ref = displayRef();
        zoomedPlots.delete(plots.zoomPos);
        zoomedPlots.delete(plots.zoomVel);
        zoomedPlots.delete(plots.zoomAcc);
        for (const arr of Object.values(zoom)) arr.length = 0;

        for (let i = 0; i < live.raw_time.length; i++) {
            const t = live.raw_time[i];
            if (t < t0Raw || t > t1Raw) continue;
            zoom.time.push(t - ref);
            zoom.est_pos.push(live.est_pos[i]);
            zoom.est_vel.push(live.est_vel[i]);
            zoom.est_acc.push(live.est_acc[i]);
            zoom.act_pos.push(live.act_pos[i]);
            zoom.act_vel.push(live.act_vel[i]);
            zoom.act_acc.push(live.act_acc[i]);
        }

        if (zoom.time.length === 0) return;   // nothing in range

        const pScale = posScale("profile");
        plots.zoomPos.setData([zoom.time, applyPos(zoom.est_pos, 0, pScale)]);
        plots.zoomVel.setData([zoom.time, zoom.est_vel]);
        plots.zoomAcc.setData([zoom.time, zoom.est_acc]);
        zoomedPlots.add(plots.zoomPos);
        zoomedPlots.add(plots.zoomVel);
        zoomedPlots.add(plots.zoomAcc);

        document.getElementById("zoom-range").textContent =
            `${(t0Raw - ref).toFixed(2)}s — ${(t1Raw - ref).toFixed(2)}s`;
        document.getElementById("zoom-idle").classList.add("hidden");
        document.getElementById("zoom-active").classList.remove("hidden");

        switchTab("zoom");
        updateSaveCsvBtn();
        requestAnimationFrame(resizePlots);
    }

    // Attach mouse handlers to every left-panel overlay.
    // selRect uses overlay-relative coords; posToVal uses plot.over-relative coords.
    overlayEls.forEach((overlay, oi) => {
        const plot = leftPlots[oi];

        overlay.addEventListener("mousedown", e => {
            if (!state.cropMode) return;
            const overRect    = plot.over.getBoundingClientRect();
            const overlayRect = overlay.getBoundingClientRect();
            const overX       = Math.max(0, Math.min(overRect.width, e.clientX - overRect.left));
            const overlayX    = e.clientX - overlayRect.left;
            const ref         = displayRef();

            cropState.startRaw      = plot.posToVal(overX, "x") + ref;
            cropState.startOverlayX = overlayX;
            cropState.activePlot    = plot;
            cropState.dragging      = true;
            updateSelectionRects(overlayX, overlayX);
        });

        overlay.addEventListener("mousemove", e => {
            if (!cropState.dragging) return;
            const overlayRect = overlay.getBoundingClientRect();
            updateSelectionRects(cropState.startOverlayX, e.clientX - overlayRect.left);
        });
    });

    // Global mouseup — fires even when the mouse is released outside the overlay.
    window.addEventListener("mouseup", e => {
        if (!cropState.dragging) return;
        cropState.dragging = false;

        const plot    = cropState.activePlot ?? plots.position;
        const overRect = plot.over.getBoundingClientRect();
        const overX   = Math.max(0, Math.min(overRect.width, e.clientX - overRect.left));
        const ref     = displayRef();
        const endRaw  = plot.posToVal(overX, "x") + ref;
        const t0 = Math.min(cropState.startRaw, endRaw);
        const t1 = Math.max(cropState.startRaw, endRaw);

        if (state.cropMode && t1 > t0) {
            applyZoom(t0, t1);
        }
        exitCropMode();
    });

    // ── Controls ──────────────────────────────────────────────────────────────

    document.getElementById("btn-auto").addEventListener("click", ev => {
        state.trackEvents = !state.trackEvents;
        ev.target.classList.toggle("active", state.trackEvents);
    });

    document.getElementById("btn-stop").addEventListener("click", () => {
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ command: "stop_experiment" }));
        } else {
            stopProfile();
        }
    });

    document.getElementById("btn-time-sync").addEventListener("click", () => {
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ command: "time_sync" }));
        } else {
            // Fallback: set reference from local data
            const last = live.raw_time[live.raw_time.length - 1];
            if (last != null) state.timeRef = last;
        }
    });

    document.getElementById("btn-pos-sync").addEventListener("click", () => {
        const act = live.act_pos[live.act_pos.length - 1];
        const est = live.est_pos[live.est_pos.length - 1];
        if (act == null || est == null || !ws || ws.readyState !== WebSocket.OPEN) return;
        ws.send(JSON.stringify({ command: "pos_sync", data: { delta_rad: act - est } }));
    });

    document.getElementById("btn-zero").addEventListener("click", () => {
        const act = live.act_pos[live.act_pos.length - 1];
        if (act == null || !ws || ws.readyState !== WebSocket.OPEN) return;
        ws.send(JSON.stringify({ command: "zero_set", data: { act_rad: act } }));
    });

    document.getElementById("btn-unit-live").addEventListener("click", ev => {
        state.livePosUnit = state.livePosUnit === "rad" ? "deg" : "rad";
        ev.target.textContent = state.livePosUnit;
        updatePosLabels();
    });

    document.getElementById("btn-unit-profile").addEventListener("click", ev => {
        state.profilePosUnit = state.profilePosUnit === "rad" ? "deg" : "rad";
        ev.target.textContent = state.profilePosUnit;
        updatePosLabels();
    });

    document.getElementById("btn-crop").addEventListener("click", () => {
        if (state.cropMode) {
            exitCropMode();
        } else {
            enterCropMode();
        }
    });

    document.getElementById("btn-reset-zoom").addEventListener("click", () => {
        zoomedPlots.delete(plots.zoomPos);
        zoomedPlots.delete(plots.zoomVel);
        zoomedPlots.delete(plots.zoomAcc);
        const pScale = posScale("profile");
        plots.zoomPos.setData([zoom.time, applyPos(zoom.est_pos, 0, pScale)]);
        plots.zoomVel.setData([zoom.time, zoom.est_vel]);
        plots.zoomAcc.setData([zoom.time, zoom.est_acc]);
    });

    document.getElementById("btn-reset-profile").addEventListener("click", () => {
        zoomedPlots.delete(plots.profilePos);
        zoomedPlots.delete(plots.profileVel);
        zoomedPlots.delete(plots.profileAcc);
        const pScale = posScale("profile");
        const targetData = profile.time.map(() =>
            state.targetRef != null ? state.targetRef * pScale : null);
        plots.profilePos.setData([profile.time,
            applyPos(profile.est_pos, 0, pScale),
            targetData]);
        plots.profileVel.setData([profile.time, profile.est_vel]);
        plots.profileAcc.setData([profile.time, profile.est_acc]);
    });

    document.getElementById("btn-clear").addEventListener("click", () => {
        for (const arr of Object.values(live))    arr.length = 0;
        for (const arr of Object.values(profile)) arr.length = 0;
        for (const arr of Object.values(zoom))    arr.length = 0;
        updateSaveCsvBtn();
        redraw();
    });

    document.getElementById("btn-pause").addEventListener("click", ev => {
        state.paused = !state.paused;
        ev.target.textContent = state.paused ? "Resume" : "Pause";
    });

    // Hide tooltip when mouse leaves any plot
    document.querySelectorAll(".plot").forEach(el => {
        el.addEventListener("mouseleave", () => tooltip.classList.add("hidden"));
    });

    // Initial layout pass
    requestAnimationFrame(resizePlots);
})();
