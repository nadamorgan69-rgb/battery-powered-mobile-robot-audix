import { Esp32HttpTransport } from "./transport_esp32.js";

const wheels = ["fl", "fr", "rl", "rr"];
const pidGroups = [
    { key: "wheel", label: "Wheel / RPM" },
    { key: "yaw", label: "Yaw / Z" },
];

const ui = {
    connectionBadge: document.getElementById("connectionBadge"),
    modeBadge: document.getElementById("modeBadge"),
    endpointValue: document.getElementById("endpointValue"),
    terminalLog: document.getElementById("terminalLog"),
    commandForm: document.getElementById("commandForm"),
    commandInput: document.getElementById("commandInput"),
    pidRows: document.getElementById("pidRows"),
    holdBadge: document.getElementById("holdBadge"),
    yawPathWarning: document.getElementById("yawPathWarning"),
};

let transport;
let latestHistory = "";

boot();

function boot() {
    buildPidRows();
    bindUi();

    transport = new Esp32HttpTransport({
        onState: renderState,
        onHistory: renderHistory,
        onStatus: renderConnection,
    });
    ui.endpointValue.textContent = transport.url();
    transport.start();
}

function bindUi() {
    document.body.addEventListener("click", (event) => {
        const commandButton = event.target.closest("[data-command]");
        if (commandButton) {
            send(commandButton.dataset.command);
            return;
        }

        const copyButton = event.target.closest("[data-copy-target]");
        if (copyButton) {
            copyCommandBlock(copyButton.dataset.copyTarget);
            return;
        }

        const pidAdjust = event.target.closest("[data-pid-adjust]");
        if (pidAdjust) {
            adjustPidInput(pidAdjust.dataset.pidGroup, pidAdjust.dataset.pidTerm, Number(pidAdjust.dataset.pidAdjust));
            return;
        }

        const pidApply = event.target.closest("[data-pid-apply]");
        if (pidApply) {
            applyPid(pidApply.dataset.pidApply);
        }
    });

    document.getElementById("forwardButton").addEventListener("click", () => {
        send(`forward ${numberValue("distanceCm", 20).toFixed(1)}`);
    });
    document.getElementById("backwardButton").addEventListener("click", () => {
        send(`backward ${numberValue("distanceCm", 20).toFixed(1)}`);
    });
    document.getElementById("rotateCcwButton").addEventListener("click", () => {
        send(`rotate ${numberValue("angleDeg", 90).toFixed(1)}`);
    });
    document.getElementById("rotateCwButton").addEventListener("click", () => {
        send(`rotate ${(-Math.abs(numberValue("angleDeg", 90))).toFixed(1)}`);
    });

    document.getElementById("sendRpmButton").addEventListener("click", () => send(buildRpmCommand("send")));
    document.getElementById("reverseRpmButton").addEventListener("click", () => send(buildRpmCommand("reverse")));

    document.getElementById("clearLogButton").addEventListener("click", () => {
        latestHistory = "";
        ui.terminalLog.textContent = "";
    });
    document.getElementById("copyLogButton").addEventListener("click", async () => {
        try {
            await navigator.clipboard.writeText(ui.terminalLog.textContent);
        } catch {
            appendLocalLog("[warn] Clipboard copy blocked by browser.");
        }
    });

    ui.commandForm.addEventListener("submit", (event) => {
        event.preventDefault();
        const command = ui.commandInput.value.trim();
        if (!command) return;
        send(command);
        ui.commandInput.value = "";
    });
}

function buildPidRows() {
    ui.pidRows.innerHTML = pidGroups.map((group) => `
        <article class="pid-row">
            <h3>${group.label}</h3>
            ${["kp", "ki", "kd"].map((term) => `
                <label>${term.toUpperCase()}
                    <div class="stepper">
                        <button type="button" data-pid-adjust="-0.05" data-pid-group="${group.key}" data-pid-term="${term}">-</button>
                        <input id="pid-${group.key}-${term}" type="number" value="0.0000" step="0.01" min="0">
                        <button type="button" data-pid-adjust="0.05" data-pid-group="${group.key}" data-pid-term="${term}">+</button>
                    </div>
                </label>
            `).join("")}
            <button class="primary" type="button" data-pid-apply="${group.key}">Apply ${group.label}</button>
        </article>
    `).join("");
}

async function send(command) {
    appendLocalLog(`[tx] ${command}`);
    const ok = await transport.send(command);
    if (!ok) appendLocalLog(`[error] command failed: ${command}`);
}

function buildRpmCommand(action) {
    const wheel = document.getElementById("rpmWheel").value;
    const duration = positiveValue("rpmDuration", 2.0);
    let rpm = numberValue("rpmValue", 60);
    if (action === "reverse") rpm = -Math.abs(rpm);
    return wheel === "all" ? `rpms all ${fmt(rpm)} ${fmt(duration)}` : `rpm ${wheel} ${fmt(rpm)} ${fmt(duration)}`;
}

function adjustPidInput(group, term, delta) {
    const input = document.getElementById(`pid-${group}-${term}`);
    input.value = Math.max(0, numberFromInput(input, 0) + delta).toFixed(4);
}

function applyPid(group) {
    const kp = numberValue(`pid-${group}-kp`, 0);
    const ki = numberValue(`pid-${group}-ki`, 0);
    const kd = numberValue(`pid-${group}-kd`, 0);
    send(`pid ${group} ${kp.toFixed(4)} ${ki.toFixed(4)} ${kd.toFixed(4)}`);
}

function renderConnection(status) {
    ui.connectionBadge.textContent = status.label || (status.connected ? "Connected" : "Offline");
    ui.connectionBadge.className = `badge ${status.connected ? "connected" : "danger"}`;
}

function renderState(state) {
    ui.modeBadge.textContent = `mode: ${state.mode?.subsystem || "unknown"}`;
    ui.modeBadge.className = state.mode?.subsystem === "base_dashboard" ? "badge connected" : "badge warning";

    setText("odomXCm", fixed(state.odom?.xCm, 2));
    setText("odomYCm", fixed(state.odom?.yCm, 2));
    setText("odomYawDeg", fixed(state.odom?.thetaDeg, 2));
    setText("odomVx", fixed(state.odom?.vx, 3));
    setText("odomVy", fixed(state.odom?.vy, 3));
    setText("odomWz", fixed(state.odom?.wz, 3));

    setText("imuHealth", state.imu?.healthy ? "healthy" : "fault");
    setText("imuYawDeg", fixed(state.imu?.yawDeg, 2));
    setText("imuGyroZ", fixed(state.imu?.gyroZ, 3));
    setText("imuAccelX", fixed(state.imu?.accelX, 3));
    setText("imuAccelY", fixed(state.imu?.accelY, 3));
    setText("imuAccelZ", fixed(state.imu?.accelZ, 3));

    const heading = state.heading || {};
    setText("headingPhase", heading.phaseName || "idle");
    setText("headingSettled", heading.settled ? "yes" : "no");
    setText("headingTargetDeg", fixed(heading.targetYawDeg, 2));
    setText("headingErrorDeg", fixed(heading.yawErrorDeg, 2));
    setText("headingCommandWz", fixed(heading.commandWz, 3));
    setText("headingStableMs", fixed(heading.stableMs, 0));
    setText("commandMotionAllowed", state.command?.motionAllowed ? "yes" : "no");
    setText("commandAgeMs", fixed(state.command?.cmdAgeMs, 0));
    setText("commandWzActual", fixed(state.command?.wz, 3));
    ui.holdBadge.textContent = heading.holdEnabled ? `hold: ${heading.phaseName || "on"}` : "hold: off";
    ui.holdBadge.className = `badge ${heading.holdEnabled ? (heading.settled ? "connected" : "warning") : ""}`;

    let allWheelTargetsZero = true;
    for (const wheel of wheels) {
        const data = state.wheels?.[wheel] || {};
        setText(`${wheel}Count`, data.count ?? 0);
        setText(`${wheel}Rpm`, fixed(data.measuredRpm, 3));
        setText(`${wheel}TargetRpm`, fixed(data.targetRpm, 3));
        setText(`${wheel}RadS`, fixed(data.measuredRadS, 3));
        setText(`${wheel}Pwm`, fixed(data.pwm, 3));
        allWheelTargetsZero = allWheelTargetsZero && Math.abs(Number(data.targetRpm) || 0) < 0.01;
    }

    const yawCommandMissingWheelTargets =
        heading.holdEnabled && Math.abs(Number(heading.commandWz) || 0) > 0.01 && allWheelTargetsZero;
    ui.yawPathWarning.classList.toggle("hidden", !yawCommandMissingWheelTargets);

    for (const group of pidGroups) {
        const gains = state.pid?.[group.key];
        if (!gains) continue;
        syncPidInput(group.key, "kp", gains.kp);
        syncPidInput(group.key, "ki", gains.ki);
        syncPidInput(group.key, "kd", gains.kd);
    }
}

function syncPidInput(group, term, value) {
    const input = document.getElementById(`pid-${group}-${term}`);
    if (document.activeElement === input) return;
    input.value = fixed(value, 4);
}

function renderHistory(text) {
    if (text === latestHistory) return;
    latestHistory = text;
    ui.terminalLog.textContent = text.trim();
    ui.terminalLog.scrollTop = ui.terminalLog.scrollHeight;
}

function appendLocalLog(line) {
    const current = ui.terminalLog.textContent;
    ui.terminalLog.textContent = `${current}${current ? "\n" : ""}${line}`;
    ui.terminalLog.scrollTop = ui.terminalLog.scrollHeight;
}

async function copyCommandBlock(id) {
    const element = document.getElementById(id);
    if (!element) return;
    const text = element.textContent.trim();
    try {
        await navigator.clipboard.writeText(text);
        appendLocalLog(`[copy] ${id}`);
    } catch {
        ui.commandInput.value = text.split("\n")[0] || "";
        appendLocalLog("[warn] Clipboard copy blocked; first command loaded.");
    }
}

function setText(id, value) {
    document.getElementById(id).textContent = value;
}

function numberValue(id, fallback) {
    return numberFromInput(document.getElementById(id), fallback);
}

function numberFromInput(input, fallback) {
    const value = Number(input.value);
    return Number.isFinite(value) ? value : fallback;
}

function positiveValue(id, fallback) {
    const value = numberValue(id, fallback);
    return value > 0 ? value : fallback;
}

function fixed(value, digits) {
    return Number.isFinite(Number(value)) ? Number(value).toFixed(digits) : Number(0).toFixed(digits);
}

function fmt(value) {
    return Number(value).toFixed(2);
}
